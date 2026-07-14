/*
   adaptive.cpp
   GrowHub32 - Adaptive Learning Mechanics Implementation
   Version: 1.2.1
   Revision: Fixed buffer overflow in readProfileJSON (off-by-one null terminator).
             Replaced magic number 1024 with PROFILE_JSON_MAX_SIZE from config.h.
             Clarified calibration variable names (offsets vs absolute timestamps).
             Added confidence score EMA smoothing.
             Added SD write failure detection.
             Softened rise/fall transition threshold to reduce false triggers.

   Profile files are stored as JSON on SD card:
   /profiles/band_18_21.json
   /profiles/band_21_24.json
   /profiles/band_24_27.json
   /profiles/band_27_30.json

   JSON structure per GH-AL-002:
   {
     "risetimeseconds": 120.5,
     "decaytimeseconds": 300.2,
     "avgrecoveryrate": 0.15,
     "confidence_score": 0.85,
     "sample_count": 45
   }
*/

#include "adaptive.h"
#include "sd_logger.h"
#include "sensors.h"
#include "rtc_handler.h"
#include "network.h"
#include <SD.h>
#include <ArduinoJson.h>

extern void network_sendAlert(const char* title, const char* message);

static BandProfile g_bandProfiles[4];
AdaptiveState g_adaptiveState;

// Calibration tracking
// NOTE: These are OFFSETS from calibrationStartTime, not absolute millis() timestamps.
// They are set to the value of 'elapsed' at the transition moment.
static float g_calibRiseStartRH = 0;
static unsigned long g_calibRiseStartElapsed = 0;   // Offset from calib start when rising began
static float g_calibFallStartRH = 0;
static unsigned long g_calibFallStartElapsed = 0;   // Offset from calib start when falling began
static bool g_calibRising = true;

// Band filename mapping
static const char* bandFiles[4] = {
  BAND_18_21_FILE,
  BAND_21_24_FILE,
  BAND_24_27_FILE,
  BAND_27_30_FILE
};

// ============================================
// Private Helpers
// ============================================

static bool sdAvailable() {
  return (SD.cardType() != CARD_NONE);
}

static bool readProfileJSON(const char* filename, BandProfile* profile) {
  if (!sdAvailable()) return false;

  if (!SD.exists(filename)) {
    Serial.print(F("[ADAPT] Profile file not found: "));
    Serial.println(filename);
    return false;
  }

  File f = SD.open(filename, FILE_READ);
  if (!f) {
    Serial.print(F("[ADAPT] Cannot open profile: "));
    Serial.println(filename);
    return false;
  }

  // Read file into buffer
  size_t fileSize = f.size();

  // Guard against oversized files, leaving room for null terminator
  if (fileSize > PROFILE_JSON_MAX_SIZE) {
    Serial.print(F("[ADAPT] Profile file too large: "));
    Serial.print(fileSize);
    Serial.print(F(" bytes (max "));
    Serial.print(PROFILE_JSON_MAX_SIZE);
    Serial.println(F(")"));
    f.close();
    return false;
  }

  // Buffer has +1 for null terminator (fixes off-by-one overflow)
  uint8_t buffer[PROFILE_JSON_MAX_SIZE + 1];
  size_t bytesRead = f.readBytes((char*)buffer, fileSize);
  buffer[bytesRead] = '\0';  // Safe: bytesRead <= PROFILE_JSON_MAX_SIZE < sizeof(buffer)
  f.close();

  // Parse JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, buffer);

  if (error) {
    Serial.print(F("[ADAPT] JSON parse error in "));
    Serial.print(filename);
    Serial.print(F(": "));
    Serial.println(error.c_str());

    // Corrupted file detected (GH-LOG-003)
    network_sendAlert("Profile Corruption",
                     ("JSON parse failed for " + String(filename)).c_str());
    return false;
  }

  // Extract values with defaults
  profile->riseTimeSeconds = doc["risetimeseconds"] | 0.0f;
  profile->decayTimeSeconds = doc["decaytimeseconds"] | 0.0f;
  profile->avgRecoveryRate = doc["avgrecoveryrate"] | 0.0f;
  profile->confidenceScore = doc["confidence_score"] | 0.0f;
  profile->sampleCount = doc["sample_count"] | 0;
  profile->valid = (profile->sampleCount > 0);

  // Clamp confidence to max
  if (profile->confidenceScore > CONFIDENCE_MAX) {
    profile->confidenceScore = CONFIDENCE_MAX;
  }

  return true;
}

static bool writeProfileJSON(const char* filename, const BandProfile* profile) {
  if (!sdAvailable()) return false;

  StaticJsonDocument<512> doc;

  doc["risetimeseconds"] = profile->riseTimeSeconds;
  doc["decaytimeseconds"] = profile->decayTimeSeconds;
  doc["avgrecoveryrate"] = profile->avgRecoveryRate;
  doc["confidence_score"] = profile->confidenceScore;
  doc["sample_count"] = profile->sampleCount;

  File f = SD.open(filename, FILE_WRITE);
  if (!f) {
    Serial.print(F("[ADAPT] Cannot write profile: "));
    Serial.println(filename);
    return false;
  }

  if (serializeJson(doc, f) == 0) {
    Serial.println(F("[ADAPT] Failed to serialize profile JSON"));
    f.close();
    return false;
  }

  f.close();
  Serial.print(F("[ADAPT] Profile saved: "));
  Serial.println(filename);
  return true;
}

// ============================================
// Public API
// ============================================

bool adaptive_init() {
  Serial.println(F("[ADAPT] Initializing adaptive learning engine..."));

  // Initialize state
  g_adaptiveState.activeBandIndex = BAND_18_21_INDEX;
  g_adaptiveState.calibrationActive = false;
  g_adaptiveState.calibrationStartTime = 0;
  g_adaptiveState.calibrationStartRH = 0;
  g_adaptiveState.calibrationPeakRH = 0;
  g_adaptiveState.calibrationLowRH = 100;
  g_adaptiveState.emaWeight = DEFAULT_EMA_WEIGHT;

  // Initialize profiles to empty
  for (uint8_t i = 0; i < 4; i++) {
    g_bandProfiles[i].riseTimeSeconds = 0;
    g_bandProfiles[i].decayTimeSeconds = 0;
    g_bandProfiles[i].avgRecoveryRate = 0;
    g_bandProfiles[i].confidenceScore = 0;
    g_bandProfiles[i].sampleCount = 0;
    g_bandProfiles[i].valid = false;
  }

  Serial.println(F("[ADAPT] Adaptive engine initialized"));
  return true;
}

bool adaptive_loadProfiles() {
  // GH-AL-001: Load 4 temperature band profiles from SD
  if (!sdAvailable()) {
    Serial.println(F("[ADAPT] SD not available - using empty profiles"));
    return false;
  }

  bool anyLoaded = false;
  for (uint8_t i = 0; i < 4; i++) {
    if (readProfileJSON(bandFiles[i], &g_bandProfiles[i])) {
      Serial.print(F("[ADAPT] Loaded profile band "));
      Serial.print(i);
      Serial.print(F(": confidence="));
      Serial.print(g_bandProfiles[i].confidenceScore, 2);
      Serial.print(F(", samples="));
      Serial.print(g_bandProfiles[i].sampleCount);
      Serial.print(F(", recovery="));
      Serial.print(g_bandProfiles[i].avgRecoveryRate, 3);
      Serial.println(F(" %/s"));
      anyLoaded = true;
    }
  }

  if (!anyLoaded) {
    Serial.println(F("[ADAPT] No profile files found - calibration needed for all bands"));
  }

  return anyLoaded;
}

bool adaptive_saveProfile(uint8_t bandIndex) {
  if (bandIndex > 3) return false;

  bool success = writeProfileJSON(bandFiles[bandIndex], &g_bandProfiles[bandIndex]);

  if (!success) {
    Serial.print(F("[ADAPT] WARNING: Failed to save profile for band "));
    Serial.println(bandIndex);
    // Profile remains valid in RAM but won't survive reboot
  }

  return success;
}

uint8_t adaptive_getCurrentBand() {
  // GH-AL-003: Select band based on live temperature
  float temp = sensors_isTemperatureValid() ?
               g_systemState.currentTemp :
               g_systemState.lastKnownGoodTemp;

  if (temp < BAND_18_21_HIGH) {
    return BAND_18_21_INDEX;
  } else if (temp < BAND_21_24_HIGH) {
    return BAND_21_24_INDEX;
  } else if (temp < BAND_24_27_HIGH) {
    return BAND_24_27_INDEX;
  } else {
    return BAND_27_30_INDEX;
  }
}

BandProfile* adaptive_getActiveProfile() {
  uint8_t band = adaptive_getCurrentBand();
  g_adaptiveState.activeBandIndex = band;
  return &g_bandProfiles[band];
}

// ============================================
// Calibration (GH-AL-004)
// ============================================

bool adaptive_startCalibration() {
  // GH-AL-004: Start 15-minute calibration sequence
  if (g_adaptiveState.calibrationActive) {
    Serial.println(F("[ADAPT] Calibration already active - ignoring request"));
    return false;
  }

  uint8_t band = adaptive_getCurrentBand();

  g_adaptiveState.calibrationActive = true;
  g_adaptiveState.calibrationStartTime = millis();
  g_adaptiveState.calibrationStartRH = sensors_isHumidityValid() ?
                                        g_systemState.currentHumidity :
                                        g_systemState.lastKnownGoodHumidity;
  g_adaptiveState.calibrationPeakRH = g_adaptiveState.calibrationStartRH;
  g_adaptiveState.calibrationLowRH = g_adaptiveState.calibrationStartRH;

  g_calibRising = true;
  g_calibRiseStartRH = g_adaptiveState.calibrationStartRH;
  g_calibRiseStartElapsed = 0;
  g_calibFallStartRH = 0;
  g_calibFallStartElapsed = 0;

  g_systemState.calibrationActive = true;

  Serial.print(F("[ADAPT] Calibration started for band "));
  Serial.print(band);
  Serial.print(F(" - initial RH: "));
  Serial.print(g_adaptiveState.calibrationStartRH, 1);
  Serial.println(F("%"));

  network_sendAlert("Calibration Started",
                   ("Band " + String(band) + " calibration in progress (15 min)").c_str());

  return true;
}

void adaptive_updateCalibration() {
  if (!g_adaptiveState.calibrationActive) return;

  unsigned long now = millis();
  unsigned long elapsed = now - g_adaptiveState.calibrationStartTime;

  // Check if calibration duration has elapsed (15 minutes)
  if (elapsed >= (CALIBRATION_DURATION_SEC * 1000UL)) {
    // Calibration complete - process results
    uint8_t band = adaptive_getCurrentBand();
    BandProfile* profile = &g_bandProfiles[band];

    // Calculate recovery metrics from rise/fall phases
    if (g_calibRiseStartElapsed > 0 && g_calibFallStartElapsed > 0) {
      // Both rise and fall detected
      // riseStartElapsed and fallStartElapsed are offsets from calibrationStartTime
      // So: riseDuration = fallStartElapsed - riseStartElapsed
      //     fallDuration = totalElapsed - fallStartElapsed
      unsigned long riseDurationMs = g_calibFallStartElapsed - g_calibRiseStartElapsed;
      unsigned long fallDurationMs = elapsed - g_calibFallStartElapsed;
      float rhDelta = g_adaptiveState.calibrationPeakRH - g_adaptiveState.calibrationLowRH;

      if (riseDurationMs > 0 && fallDurationMs > 0 && rhDelta > 0.5f) {
        float riseRate = rhDelta / (riseDurationMs / 1000.0f);
        float decayRate = rhDelta / (fallDurationMs / 1000.0f);

        // GH-AL-005: EMA smooth with existing data
        if (profile->valid) {
          profile->riseTimeSeconds = adaptive_applyEMA(riseDurationMs / 1000.0f, profile->riseTimeSeconds);
          profile->decayTimeSeconds = adaptive_applyEMA(fallDurationMs / 1000.0f, profile->decayTimeSeconds);
          profile->avgRecoveryRate = adaptive_applyEMA(riseRate, profile->avgRecoveryRate);
        } else {
          // First calibration for this band - use raw values
          profile->riseTimeSeconds = riseDurationMs / 1000.0f;
          profile->decayTimeSeconds = fallDurationMs / 1000.0f;
          profile->avgRecoveryRate = riseRate;
        }

        profile->sampleCount++;
        profile->valid = true;

        // GH-AL-006: Update confidence score with EMA smoothing for stability
        float rawConfidence = adaptive_updateConfidence(band);
        if (profile->sampleCount > 1) {
          profile->confidenceScore = adaptive_applyEMA(rawConfidence, profile->confidenceScore);
        } else {
          profile->confidenceScore = rawConfidence;
        }

        // Save to SD (check for write failure)
        if (!adaptive_saveProfile(band)) {
          Serial.println(F("[ADAPT] WARNING: Profile may not persist after reboot"));
        }

        Serial.println(F(""));
        Serial.println(F("============================================"));
        Serial.println(F("[ADAPT] Calibration complete!"));
        Serial.print(F("  Band: ")); Serial.println(band);
        Serial.print(F("  Rise time: ")); Serial.print(profile->riseTimeSeconds, 1); Serial.println(F("s"));
        Serial.print(F("  Decay time: ")); Serial.print(profile->decayTimeSeconds, 1); Serial.println(F("s"));
        Serial.print(F("  Recovery rate: ")); Serial.print(profile->avgRecoveryRate, 3); Serial.println(F(" %/s"));
        Serial.print(F("  Confidence: ")); Serial.print(profile->confidenceScore, 2);
        Serial.print(F("  (raw: ")); Serial.print(rawConfidence, 2); Serial.println(F(")"));
        Serial.print(F("  Total samples: ")); Serial.println(profile->sampleCount);
        Serial.println(F("============================================"));
        Serial.println(F(""));

        network_sendAlert("Calibration Complete",
                         ("Band " + String(band) + " calibrated. Confidence: " +
                          String(profile->confidenceScore, 2)).c_str());
      } else {
        Serial.println(F("[ADAPT] Calibration had insufficient RH variation - results discarded"));
        Serial.print(F("  RH delta: ")); Serial.print(rhDelta, 1);
        Serial.print(F("%, Rise: ")); Serial.print(riseDurationMs);
        Serial.print(F("ms, Fall: ")); Serial.println(fallDurationMs);
      }
    } else {
      Serial.println(F("[ADAPT] Calibration incomplete - rise and fall phases not both detected"));
    }

    // End calibration regardless of outcome
    g_adaptiveState.calibrationActive = false;
    g_systemState.calibrationActive = false;
    return;
  }

  // --- During calibration: track RH peaks and valleys ---
  float currentRH = sensors_isHumidityValid() ?
                    g_systemState.currentHumidity :
                    g_systemState.lastKnownGoodHumidity;

  // Track absolute peak and low
  if (currentRH > g_adaptiveState.calibrationPeakRH) {
    g_adaptiveState.calibrationPeakRH = currentRH;
  }
  if (currentRH < g_adaptiveState.calibrationLowRH) {
    g_adaptiveState.calibrationLowRH = currentRH;
  }

  // Detect rise-to-fall transition
  // Uses 2.5% threshold to avoid false triggers from sensor noise
  if (g_calibRising && currentRH < g_adaptiveState.calibrationPeakRH - 2.5f) {
    // Transitioned from rising to falling
    g_calibRising = false;
    g_calibFallStartRH = currentRH;
    g_calibFallStartElapsed = elapsed;  // Offset from calibration start

    Serial.print(F("[ADAPT] Calibration: Detected FALL phase at elapsed "));
    Serial.print(elapsed / 1000);
    Serial.print(F("s, RH: "));
    Serial.println(currentRH, 1);
  }
}

bool adaptive_isCalibrating() {
  return g_adaptiveState.calibrationActive;
}

void adaptive_cancelCalibration() {
  g_adaptiveState.calibrationActive = false;
  g_systemState.calibrationActive = false;
  Serial.println(F("[ADAPT] Calibration cancelled by user"));
}

unsigned long adaptive_getCalibrationStartTime() {
  return g_adaptiveState.calibrationStartTime;
}

// ============================================
// EMA Smoothing (GH-AL-005)
// ============================================

float adaptive_applyEMA(float rawValue, float storedHistorical) {
  // GH-AL-005: Exponential Moving Average
  // new_value = (weight * raw_reading) + ((1 - weight) * stored_historical)
  float weight = g_adaptiveState.emaWeight;
  return (weight * rawValue) + ((1.0f - weight) * storedHistorical);
}

// ============================================
// Confidence Score (GH-AL-006)
// ============================================

float adaptive_updateConfidence(uint8_t bandIndex) {
  // GH-AL-006: Confidence = min(0.90, 1.0 - (1.0 / (1.0 + samples_count)))
  if (bandIndex > 3) return 0;

  BandProfile* profile = &g_bandProfiles[bandIndex];
  float confidence = 1.0f - (1.0f / (1.0f + (float)profile->sampleCount));

  // Clamp to max
  if (confidence > CONFIDENCE_MAX) {
    confidence = CONFIDENCE_MAX;
  }

  return confidence;
}

// ============================================
// Recovery Projection (GH-UI-004)
// ============================================

float adaptive_projectRecoveryTime(float deltaToTarget) {
  // GH-UI-004: projected_recovery_seconds = delta_to_target / (avgrecoveryrate * confidence_score)
  BandProfile* profile = adaptive_getActiveProfile();

  if (!profile->valid || profile->avgRecoveryRate <= 0 || profile->confidenceScore <= 0) {
    // No valid data - return conservative estimate (1% per minute)
    return deltaToTarget * 60.0f;
  }

  float effectiveRate = profile->avgRecoveryRate * profile->confidenceScore;
  if (effectiveRate <= 0.0001f) return 9999.0f;  // Avoid division by near-zero

  return deltaToTarget / effectiveRate;
}

// ============================================
// EMA Weight Management
// ============================================

void adaptive_setEMAWeight(float weight) {
  if (weight < EMA_WEIGHT_MIN) weight = EMA_WEIGHT_MIN;
  if (weight > EMA_WEIGHT_MAX) weight = EMA_WEIGHT_MAX;
  g_adaptiveState.emaWeight = weight;

  Serial.print(F("[ADAPT] EMA weight set to: "));
  Serial.print(weight, 2);
  Serial.print(F(" ("));
  Serial.print((int)(weight * 100));
  Serial.println(F("% new data)"));
}

float adaptive_getEMAWeight() {
  return g_adaptiveState.emaWeight;
}
