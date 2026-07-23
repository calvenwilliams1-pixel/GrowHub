/*
   adaptive.cpp
   GrowHub32 - Adaptive Learning Mechanics Implementation
   Version: 1.3.1
   Revision: Active calibration phase state machine (v1.3).
             Calibration now actively controls HOH + Air Assist through
             STABILIZE → RISE → DECAY → COMPLETE phases.
             Slope-based plateau detection replaces passive rise/fall detection.
             Safety abort on sensor fault or emergency humidity levels.
             Fixed endRisePhase() timestamp bug (capture riseStartTime before overwrite).
             PID frozen during calibration via automation_suspendPID().
             PID re-enabled with reset via automation_resumePID().
             g_adaptiveState calibration writes protected by g_stateMux.
             canStartCalibration() enforces night mode 30-minute buffer.
             Minimum expected RH rise validation (CALIBRATION_MIN_EXPECTED_RISE_PCT).
             Configurable fallback in adaptive_projectRecoveryTime().
             g_systemState.calibrationActive writes protected by g_stateMux.
             Fixed buffer overflow in readProfileJSON (off-by-one null terminator).
             Replaced String concatenation in readProfileJSON with snprintf.
             Added confidence score EMA smoothing.
             Added SD write failure detection.

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
#include "relay_manager.h"
#include "pid_controller.h"
#include "automation.h"
#include "system_state.h"
#include <SD.h>
#include <ArduinoJson.h>

extern void network_sendAlert(const char* title, const char* message);
extern portMUX_TYPE g_stateMux;

static BandProfile g_bandProfiles[4];
AdaptiveState g_adaptiveState;

// Calibration tracking
static bool g_calibRiseDetected = false;
static unsigned long g_calibRiseStartElapsed = 0;
static unsigned long g_calibFallStartElapsed = 0;
static bool g_calibRising = true;

// Forward declarations
static void endRisePhase(unsigned long now);
static bool canStartCalibration();

// Band filename mapping
static const char* bandFiles[4] = {
  BAND_18_21_FILE,
  BAND_21_24_FILE,
  BAND_24_27_FILE,
  BAND_27_30_FILE
};

// ============================================================
// Private Helpers
// ============================================================

static bool sdAvailable() {
  return (SD.cardType() != CARD_NONE);
}

/**
 * @brief Check whether calibration is allowed to start right now.
 * 
 * Calibration requires:
 *   - Not already calibrating
 *   - Not in night mode
 *   - At least 30 minutes until night mode starts
 *     (calibration takes 20 minutes, need 10 min margin)
 *   - No sensor faults
 * 
 * @return true if calibration can start, false otherwise
 */
static bool canStartCalibration() {
    if (g_adaptiveState.calibrationActive) {
        Serial.println(F("[ADAPT] Calibration already active"));
        return false;
    }

    int minutesUntilNight = rtc_minutesUntilNightMode();
    
    if (minutesUntilNight < 0) {
        Serial.println(F("[ADAPT] RTC unavailable — cannot verify night mode status"));
        return false;
    }
    
    if (minutesUntilNight == 0) {
        Serial.println(F("[ADAPT] Night mode is active — calibration not allowed"));
        return false;
    }
    
    if (minutesUntilNight < 30) {
        Serial.print(F("[ADAPT] Only "));
        Serial.print(minutesUntilNight);
        Serial.println(F(" minutes until night mode — calibration not allowed (need 30)"));
        return false;
    }

    if (g_systemState.tempSensorFault || g_systemState.humiditySensorFault) {
        Serial.println(F("[ADAPT] Sensor fault active — calibration not allowed"));
        return false;
    }

    return true;
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

  size_t fileSize = f.size();

  if (fileSize > PROFILE_JSON_MAX_SIZE) {
    Serial.print(F("[ADAPT] Profile file too large: "));
    Serial.print(fileSize);
    Serial.print(F(" bytes (max "));
    Serial.print(PROFILE_JSON_MAX_SIZE);
    Serial.println(F(")"));
    f.close();
    return false;
  }

  uint8_t buffer[PROFILE_JSON_MAX_SIZE + 1];
  size_t bytesRead = f.readBytes((char*)buffer, fileSize);
  buffer[bytesRead] = '\0';
  f.close();

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, buffer);

  if (error) {
    Serial.print(F("[ADAPT] JSON parse error in "));
    Serial.print(filename);
    Serial.print(F(": "));
    Serial.println(error.c_str());

    char alert[128];
    snprintf(alert, sizeof(alert), "JSON parse failed for %s", filename);
    network_sendAlert("Profile Corruption", alert);
    return false;
  }

  BandProfile tempProfile;
  memset(&tempProfile, 0, sizeof(BandProfile));

  tempProfile.riseTimeSeconds = doc["risetimeseconds"] | 0.0f;
  tempProfile.decayTimeSeconds = doc["decaytimeseconds"] | 0.0f;
  tempProfile.avgRecoveryRate = doc["avgrecoveryrate"] | 0.0f;
  tempProfile.confidenceScore = doc["confidence_score"] | 0.0f;
  tempProfile.sampleCount = doc["sample_count"] | 0;
  tempProfile.valid = (tempProfile.sampleCount > 0);

  if (tempProfile.confidenceScore > CONFIDENCE_MAX) {
    tempProfile.confidenceScore = CONFIDENCE_MAX;
  }

  *profile = tempProfile;

  return true;
}

static bool writeProfileJSON(const char* filename, const BandProfile* profile) {
  if (!sdAvailable()) return false;

  if (SD.exists(filename)) {
    SD.remove(filename);
  }

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

  size_t bytes = serializeJson(doc, f);
  f.close();

  if (bytes == 0) {
    Serial.println(F("[ADAPT] Failed to serialize profile JSON"));
    return false;
  }

  Serial.print(F("[ADAPT] Profile saved: "));
  Serial.println(filename);
  return true;
}

// ============================================================
// Public API
// ============================================================

bool adaptive_init() {
  Serial.println(F("[ADAPT] Initializing adaptive learning engine..."));

  g_adaptiveState.activeBandIndex = BAND_18_21_INDEX;
  g_adaptiveState.calibrationBand = BAND_18_21_INDEX;
  g_adaptiveState.calibrationActive = false;
  g_adaptiveState.calibrationStartTime = 0;
  g_adaptiveState.calibrationStartRH = 0;
  g_adaptiveState.calibrationPeakRH = 0;
  g_adaptiveState.calibrationLowRH = 100;
  g_adaptiveState.emaWeight = DEFAULT_EMA_WEIGHT;

  g_adaptiveState.calibPhase = CALIB_PHASE_STABILIZE;
  g_adaptiveState.calibPhaseStart = 0;
  g_adaptiveState.calibPhaseBaselineRH = 0;
  g_adaptiveState.calibPhasePlateauDetected = false;
  g_adaptiveState.calibPlateauSampleStart = 0;
  g_adaptiveState.calibPlateauSampleRH = 0.0f;

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
  }

  return success;
}

uint8_t adaptive_getCurrentBand() {
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

// ============================================================
// Active Calibration (GH-AL-004) — v1.3.1
// ============================================================

bool adaptive_startCalibration() {
  if (!canStartCalibration()) {
    return false;
  }

  g_adaptiveState.calibrationBand = adaptive_getCurrentBand();
  uint8_t band = g_adaptiveState.calibrationBand;

  // Freeze PID controller during calibration.
  // Calibration directly controls HOH + Air Assist; PID must not interfere.
  automation_suspendPID();

  // Initialize all calibration state under mutex — Web UI may read concurrently
  portENTER_CRITICAL(&g_stateMux);
  g_adaptiveState.calibrationActive = true;
  g_adaptiveState.calibrationStartTime = millis();
  g_adaptiveState.calibrationStartRH = sensors_isHumidityValid() ?
      g_systemState.currentHumidity : g_systemState.lastKnownGoodHumidity;
  g_adaptiveState.calibPhase = CALIB_PHASE_STABILIZE;
  g_adaptiveState.calibPhaseStart = millis();
  g_adaptiveState.calibPhaseBaselineRH = g_adaptiveState.calibrationStartRH;
  g_adaptiveState.calibPhasePlateauDetected = false;
  g_adaptiveState.calibPlateauSampleStart = 0;
  g_adaptiveState.calibPlateauSampleRH = 0.0f;
  g_adaptiveState.calibrationPeakRH = g_adaptiveState.calibrationStartRH;
  g_adaptiveState.calibrationLowRH = g_adaptiveState.calibrationStartRH;
  g_systemState.calibrationActive = true;
  portEXIT_CRITICAL(&g_stateMux);

  // Reset detection variables (local to this task, no mutex needed)
  g_calibRiseDetected = false;
  g_calibRiseStartElapsed = 0;
  g_calibFallStartElapsed = 0;
  g_calibRising = true;

  // Force all humidity actuators OFF for stabilization
  relayManager_setRelay(RELAY_HOH, false);
  relayManager_setRelay(RELAY_AIR_ASSIST, false);

  Serial.print(F("[ADAPT] Active calibration started for band "));
  Serial.print(band);
  Serial.print(F(" - Phase: STABILIZE ("));
  Serial.print(CALIBRATION_STABILIZE_SEC);
  Serial.println(F("s)"));
  Serial.print(F("[ADAPT] Calibration start RH: "));
  Serial.println(g_adaptiveState.calibrationStartRH, 1);

  {
    char alertBody[64];
    snprintf(alertBody, sizeof(alertBody),
             "Band %u active calibration in progress.", band);
    network_sendAlert("Calibration Started", alertBody);
  }

  return true;
}

void adaptive_updateCalibration() {
  if (!g_adaptiveState.calibrationActive) return;

  // Safety abort: sensor fault
  if (g_systemState.tempSensorFault || g_systemState.humiditySensorFault) {
    Serial.println(F("[ADAPT] Calibration ABORTED - sensor fault detected"));
    adaptive_cancelCalibration();
    network_sendAlert("Calibration Aborted", "Sensor fault detected during calibration.");
    return;
  }

  float currentRH = sensors_isHumidityValid() ?
      g_systemState.currentHumidity : g_systemState.lastKnownGoodHumidity;

  // Emergency stop: humidity exceeds safe ceiling
  if (currentRH > SAFETY_HUM_CEILING_HARD) {
    Serial.println(F("[ADAPT] Calibration ABORTED - humidity exceeds safe limit"));
    adaptive_cancelCalibration();
    relayManager_setRelay(RELAY_HOH, false);
    relayManager_setRelay(RELAY_AIR_ASSIST, false);
    network_sendAlert("Calibration Aborted", "Humidity exceeded safe ceiling during calibration.");
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - g_adaptiveState.calibrationStartTime;

  // Track absolute peak and low regardless of phase
  if (currentRH > g_adaptiveState.calibrationPeakRH) {
    g_adaptiveState.calibrationPeakRH = currentRH;
  }
  if (currentRH < g_adaptiveState.calibrationLowRH) {
    g_adaptiveState.calibrationLowRH = currentRH;
  }

  unsigned long phaseElapsed = now - g_adaptiveState.calibPhaseStart;

  switch (g_adaptiveState.calibPhase) {

    case CALIB_PHASE_STABILIZE:
      if (phaseElapsed >= CALIBRATION_STABILIZE_SEC * 1000UL) {
        g_adaptiveState.calibPhase = CALIB_PHASE_RISE;
        g_adaptiveState.calibPhaseStart = now;
        g_adaptiveState.calibPhaseBaselineRH = currentRH;

        // Calibration directly controls relays — bypasses applyRelayProtection()
        // because calibration needs immediate response, not min pulse timing.
        relayManager_setRelay(RELAY_HOH, true);
        relayManager_setRelay(RELAY_AIR_ASSIST, true);

        Serial.println(F("[ADAPT] Calibration Phase: RISE — HOH + Air Assist ON"));
      }
      break;

    case CALIB_PHASE_RISE: {
      if (g_adaptiveState.calibPlateauSampleStart == 0) {
        g_adaptiveState.calibPlateauSampleStart = now;
        g_adaptiveState.calibPlateauSampleRH = currentRH;
      } else if (now - g_adaptiveState.calibPlateauSampleStart >=
                 CALIBRATION_PLATEAU_WINDOW_SEC * 1000UL) {
        float deltaRH = currentRH - g_adaptiveState.calibPlateauSampleRH;
        float deltaMinutes = (now - g_adaptiveState.calibPlateauSampleStart) / 60000.0f;

        if (deltaMinutes > 0.0f) {
          float slopePerMinute = fabs(deltaRH) / deltaMinutes;

          if (slopePerMinute < CALIBRATION_PLATEAU_SLOPE_PCT) {
            if (!g_adaptiveState.calibPhasePlateauDetected) {
              g_adaptiveState.calibPhasePlateauDetected = true;
              g_adaptiveState.calibPlateauSampleStart = now;
            } else if (now - g_adaptiveState.calibPlateauSampleStart >=
                       CALIBRATION_PLATEAU_CONFIRM_SEC * 1000UL) {
              Serial.println(F("[ADAPT] Calibration: Plateau confirmed — ending RISE phase"));
              endRisePhase(now);
            }
          } else {
            g_adaptiveState.calibPhasePlateauDetected = false;
            g_adaptiveState.calibPlateauSampleStart = now;
            g_adaptiveState.calibPlateauSampleRH = currentRH;
          }
        }
      }

      if (phaseElapsed >= CALIBRATION_RISE_MAX_SEC * 1000UL) {
        Serial.println(F("[ADAPT] Calibration: RISE phase timeout — maximum time reached"));
        endRisePhase(now);
      }
      break;
    }

    case CALIB_PHASE_DECAY:
      if (elapsed >= CALIBRATION_TOTAL_SEC * 1000UL) {
        g_adaptiveState.calibPhase = CALIB_PHASE_COMPLETE;

        uint8_t band = g_adaptiveState.calibrationBand;
        BandProfile* profile = &g_bandProfiles[band];

        if (g_calibRiseDetected && g_calibRiseStartElapsed > 0 && g_calibFallStartElapsed > 0) {
          unsigned long riseDurationMs = g_calibFallStartElapsed - g_calibRiseStartElapsed;
          unsigned long fallDurationMs = elapsed - g_calibFallStartElapsed;
          float rhDelta = g_adaptiveState.calibrationPeakRH - g_adaptiveState.calibrationLowRH;

          if (riseDurationMs > 0 && fallDurationMs > 0 &&
              rhDelta > CALIBRATION_MIN_RH_DELTA &&
              rhDelta >= CALIBRATION_MIN_EXPECTED_RISE_PCT) {
            float riseRate = rhDelta / (riseDurationMs / 1000.0f);
            float decayRate = rhDelta / (fallDurationMs / 1000.0f);

            if (profile->valid) {
              profile->riseTimeSeconds = adaptive_applyEMA(riseDurationMs / 1000.0f, profile->riseTimeSeconds);
              profile->decayTimeSeconds = adaptive_applyEMA(fallDurationMs / 1000.0f, profile->decayTimeSeconds);
              profile->avgRecoveryRate = adaptive_applyEMA(riseRate, profile->avgRecoveryRate);
            } else {
              profile->riseTimeSeconds = riseDurationMs / 1000.0f;
              profile->decayTimeSeconds = fallDurationMs / 1000.0f;
              profile->avgRecoveryRate = riseRate;
            }

            profile->sampleCount++;
            profile->valid = true;

            float rawConfidence = adaptive_updateConfidence(band);
            profile->confidenceScore = (profile->sampleCount > 1) ?
                adaptive_applyEMA(rawConfidence, profile->confidenceScore) : rawConfidence;

            if (!adaptive_saveProfile(band)) {
              Serial.println(F("[ADAPT] WARNING: Profile may not persist after reboot"));
            }

            Serial.println(F(""));
            Serial.println(F("============================================"));
            Serial.println(F("[ADAPT] Active calibration complete!"));
            Serial.print(F("  Band: ")); Serial.println(band);
            Serial.print(F("  Rise time: ")); Serial.print(profile->riseTimeSeconds, 1); Serial.println(F("s"));
            Serial.print(F("  Decay time: ")); Serial.print(profile->decayTimeSeconds, 1); Serial.println(F("s"));
            Serial.print(F("  Recovery rate: ")); Serial.print(profile->avgRecoveryRate, 3); Serial.println(F(" %/s"));
            Serial.print(F("  Confidence: ")); Serial.print(profile->confidenceScore, 2);
            Serial.print(F("  (raw: ")); Serial.print(rawConfidence, 2); Serial.println(F(")"));
            Serial.print(F("  Total samples: ")); Serial.println(profile->sampleCount);
            Serial.println(F("============================================"));
            Serial.println(F(""));

            {
              char alertBody[64];
              snprintf(alertBody, sizeof(alertBody),
                       "Band %u calibrated. Confidence: %.2f",
                       band, profile->confidenceScore);
              network_sendAlert("Calibration Complete", alertBody);
            }
          } else {
            Serial.println(F("[ADAPT] Calibration had insufficient RH variation - results discarded"));
            Serial.print(F("  RH delta: ")); Serial.print(rhDelta, 1);
            Serial.print(F("% (need >")); Serial.print(CALIBRATION_MIN_RH_DELTA, 1);
            Serial.print(F("% for validity, >")); Serial.print(CALIBRATION_MIN_EXPECTED_RISE_PCT, 1);
            Serial.print(F("% for useful data), Rise: ")); Serial.print(riseDurationMs);
            Serial.print(F("ms, Fall: ")); Serial.print(fallDurationMs);
            Serial.println(F("ms"));

            if (rhDelta < CALIBRATION_MIN_EXPECTED_RISE_PCT) {
              char alertBody[96];
              snprintf(alertBody, sizeof(alertBody),
                       "RH rose only %.1f%% during calibration. "
                       "Check humidifier, water reservoir, and air assist.",
                       rhDelta);
              network_sendAlert("Calibration Failed - Weak Response", alertBody);
            } else {
              char alertBody[96];
              snprintf(alertBody, sizeof(alertBody),
                       "RH varied only %.1f%% during calibration. "
                       "Need >%.1f%% swing.",
                       rhDelta, CALIBRATION_MIN_RH_DELTA);
              network_sendAlert("Calibration Failed - Low Variation", alertBody);
            }
          }
        } else {
          Serial.println(F("[ADAPT] Calibration incomplete - rise and fall phases not both detected"));
          Serial.print(F("  Rise detected: ")); Serial.print(g_calibRiseDetected ? "YES" : "NO");
          Serial.print(F(", Fall detected: ")); Serial.println(g_calibFallStartElapsed > 0 ? "YES" : "NO");

          network_sendAlert("Calibration Incomplete",
                           "Rise and fall phases not both detected.");
        }

        // Re-enable PID with fresh state after calibration completes
        automation_resumePID();

        portENTER_CRITICAL(&g_stateMux);
        g_systemState.calibrationActive = false;
        portEXIT_CRITICAL(&g_stateMux);
      }
      break;

    case CALIB_PHASE_COMPLETE:
      break;
  }
}

// ============================================================
// Phase Transition Helper (v1.3.1 — timestamp bug fixed)
// ============================================================

static void endRisePhase(unsigned long now) {
  // Capture RISE phase start time BEFORE overwriting calibPhaseStart.
  // Without this capture, riseDurationMs would always be zero and
  // calibration would silently fail every time.
  unsigned long riseStartTime = g_adaptiveState.calibPhaseStart;

  g_adaptiveState.calibPhase = CALIB_PHASE_DECAY;
  g_adaptiveState.calibPhaseStart = now;

  // Calibration directly controls relays — bypasses applyRelayProtection()
  relayManager_setRelay(RELAY_HOH, false);
  relayManager_setRelay(RELAY_AIR_ASSIST, false);

  g_calibRising = false;
  g_calibFallStartElapsed = now - g_adaptiveState.calibrationStartTime;

  g_calibRiseDetected = true;
  g_calibRiseStartElapsed = riseStartTime - g_adaptiveState.calibrationStartTime;

  Serial.println(F("[ADAPT] Calibration Phase: DECAY — All actuators OFF"));
}

bool adaptive_isCalibrating() {
  return g_adaptiveState.calibrationActive;
}

void adaptive_cancelCalibration() {
  // Clear calibration state and system mirror under mutex
  portENTER_CRITICAL(&g_stateMux);
  g_adaptiveState.calibrationActive = false;
  g_systemState.calibrationActive = false;
  portEXIT_CRITICAL(&g_stateMux);

  // Ensure actuators are turned off
  relayManager_setRelay(RELAY_HOH, false);
  relayManager_setRelay(RELAY_AIR_ASSIST, false);

  // Re-enable PID with fresh state after calibration abort
  automation_resumePID();

  Serial.println(F("[ADAPT] Calibration cancelled by user"));
}

unsigned long adaptive_getCalibrationStartTime() {
  return g_adaptiveState.calibrationStartTime;
}

// ============================================================
// EMA Smoothing (GH-AL-005)
// ============================================================

float adaptive_applyEMA(float rawValue, float storedHistorical) {
  float weight = g_adaptiveState.emaWeight;
  return (weight * rawValue) + ((1.0f - weight) * storedHistorical);
}

// ============================================================
// Confidence Score (GH-AL-006)
// ============================================================

float adaptive_updateConfidence(uint8_t bandIndex) {
  if (bandIndex > 3) return 0;

  BandProfile* profile = &g_bandProfiles[bandIndex];
  float confidence = 1.0f - (1.0f / (1.0f + (float)profile->sampleCount));

  if (confidence > CONFIDENCE_MAX) {
    confidence = CONFIDENCE_MAX;
  }

  return confidence;
}

// ============================================================
// Recovery Projection (GH-UI-004)
// ============================================================

float adaptive_projectRecoveryTime(float deltaToTarget) {
  if (deltaToTarget <= 0.0f) return 0.0f;

  BandProfile* profile = adaptive_getActiveProfile();

  if (!profile->valid || profile->avgRecoveryRate <= 0 || profile->confidenceScore <= 0) {
    if (DEFAULT_RECOVERY_RATE_PCT_PER_SEC > 0.0f) {
      return deltaToTarget / DEFAULT_RECOVERY_RATE_PCT_PER_SEC;
    }
    return deltaToTarget * 60.0f;
  }

  float effectiveRate = profile->avgRecoveryRate * profile->confidenceScore;
  if (effectiveRate <= 0.0001f) return 9999.0f;

  return deltaToTarget / effectiveRate;
}

// ============================================================
// EMA Weight Management
// ============================================================

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
