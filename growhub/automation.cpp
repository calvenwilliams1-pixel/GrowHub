/*
   automation.cpp
   GrowHub32 - Environmental Automation Logic Implementation
   Version: 1.2.3
   Revision: Per-subsystem manual overrides (humidity/CO2 independent).
             Threshold cross-validation with relational checks.
             Night mode force-off logging without false provenance claims.
             Automation loops skip when their specific override is active.

   Relay mapping:
   - RELAY_HOH (0):        GPIO 13 - Humidifier
   - RELAY_AIR_ASSIST (1): GPIO 12 - Air Assist Valve
   - RELAY_EXHAUST (2):    GPIO 14 - Exhaust Fan
   - RELAY_COMPRESSOR (3): GPIO 27 - Air Compressor
*/

#include "automation.h"
#include "relay_manager.h"
#include "sensors.h"
#include "rtc_handler.h"

// External declarations
extern void network_sendAlert(const char* title, const char* message);

static AutomationThresholds g_thresholds;

// Air Assist burst timing state
static bool g_airAssistBurstActive = false;
static unsigned long g_airAssistBurstStart = 0;
static bool g_airAssistInOnPhase = false;

// Manual override state (per subsystem)
static bool g_humidityOverrideActive = false;
static unsigned long g_humidityOverrideStart = 0;
static bool g_co2OverrideActive = false;
static unsigned long g_co2OverrideStart = 0;

// ============================================
// Initialization
// ============================================

void automation_init() {
  automation_loadDefaults();
  Serial.println(F("[AUTO] Automation engine initialized with defaults"));
}

void automation_loadDefaults() {
  g_thresholds.humHoHFloor = DEFAULT_HUM_HOH_FLOOR;
  g_thresholds.humAssistFloor = DEFAULT_HUM_ASSIST_FLOOR;
  g_thresholds.humCeiling = DEFAULT_HUM_CEILING;
  g_thresholds.assistOnSec = DEFAULT_HUM_ASSIST_ON_SEC;
  g_thresholds.assistOffSec = DEFAULT_HUM_ASSIST_OFF_SEC;
  g_thresholds.co2HighLimit = DEFAULT_CO2_HIGH_LIMIT;
  g_thresholds.co2LowTarget = DEFAULT_CO2_LOW_TARGET;
  g_thresholds.co2Emergency = DEFAULT_CO2_EMERGENCY;

  Serial.println(F("[AUTO] Loaded default thresholds:"));
  Serial.print(F("  HOH Floor: ")); Serial.print(g_thresholds.humHoHFloor); Serial.println(F("%"));
  Serial.print(F("  Assist Floor: ")); Serial.print(g_thresholds.humAssistFloor); Serial.println(F("%"));
  Serial.print(F("  Ceiling: ")); Serial.print(g_thresholds.humCeiling); Serial.println(F("%"));
  Serial.print(F("  CO2 High: ")); Serial.print(g_thresholds.co2HighLimit); Serial.println(F(" ppm"));
  Serial.print(F("  CO2 Low: ")); Serial.print(g_thresholds.co2LowTarget); Serial.println(F(" ppm"));
  Serial.print(F("  CO2 Emergency: ")); Serial.print(g_thresholds.co2Emergency); Serial.println(F(" ppm"));
}

// ============================================
// Humidity Automation Loop (GH-HUM)
// ============================================

void automation_runHumidityLoop() {
  // Skip if calibration is active
  if (g_systemState.calibrationActive) {
    return;
  }

  // Skip if humidity override is active (manual control from web UI)
  if (automation_isHumidityOverrideActive()) {
    return;
  }

  // Get current humidity (use last known good if sensor fault)
  float currentHumidity = sensors_isHumidityValid() ?
                          g_systemState.currentHumidity :
                          g_systemState.lastKnownGoodHumidity;

  // --- CEILING CHECK: At or above ceiling, turn everything OFF ---
  // GH-HUM-005
  if (currentHumidity >= g_thresholds.humCeiling) {
    if (relayManager_isRelayOn(RELAY_HOH)) {
      Serial.println(F("[AUTO-HUM] Ceiling reached - turning OFF HOH"));
      relayManager_setRelay(RELAY_HOH, false);
    }
    if (g_airAssistBurstActive || relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
      Serial.println(F("[AUTO-HUM] Ceiling reached - turning OFF Air Assist"));
      relayManager_setRelay(RELAY_AIR_ASSIST, false);
      g_airAssistBurstActive = false;
      g_airAssistInOnPhase = false;
    }
    return;
  }

  // --- HOH FLOOR CHECK: Below HOH floor, turn ON humidifier ---
  // GH-HUM-002
  if (currentHumidity < g_thresholds.humHoHFloor) {
    if (!relayManager_isRelayOn(RELAY_HOH)) {
      Serial.print(F("[AUTO-HUM] Below HOH floor ("));
      Serial.print(currentHumidity, 1);
      Serial.print(F(" < "));
      Serial.print(g_thresholds.humHoHFloor);
      Serial.println(F("%) - turning ON HOH"));
      relayManager_setRelay(RELAY_HOH, true);
    }
  } else {
    // Above HOH floor but below ceiling - turn OFF HOH if it was running
    if (relayManager_isRelayOn(RELAY_HOH)) {
      Serial.println(F("[AUTO-HUM] Above HOH floor - turning OFF HOH"));
      relayManager_setRelay(RELAY_HOH, false);
    }
  }

  // --- AIR ASSIST FLOOR CHECK ---
  // GH-HUM-003, GH-HUM-004
  if (currentHumidity < g_thresholds.humAssistFloor) {
    // Need air assist - run in burst cycles
    if (!g_airAssistBurstActive) {
      g_airAssistBurstActive = true;
      g_airAssistInOnPhase = true;
      g_airAssistBurstStart = millis();

      Serial.print(F("[AUTO-HUM] Below Assist floor ("));
      Serial.print(currentHumidity, 1);
      Serial.print(F(" < "));
      Serial.print(g_thresholds.humAssistFloor);
      Serial.println(F("%) - starting Air Assist bursts"));
      relayManager_setRelay(RELAY_AIR_ASSIST, true);
    }

    // Manage burst timing
    unsigned long now = millis();
    if (g_airAssistInOnPhase) {
      unsigned long onElapsed = now - g_airAssistBurstStart;
      if (onElapsed >= (g_thresholds.assistOnSec * 1000UL)) {
        g_airAssistInOnPhase = false;
        g_airAssistBurstStart = now;
        relayManager_setRelay(RELAY_AIR_ASSIST, false);
        Serial.println(F("[AUTO-HUM] Air Assist ON burst complete - OFF phase"));
      }
    } else {
      unsigned long offElapsed = now - g_airAssistBurstStart;
      if (offElapsed >= (g_thresholds.assistOffSec * 1000UL)) {
        g_airAssistInOnPhase = true;
        g_airAssistBurstStart = now;
        relayManager_setRelay(RELAY_AIR_ASSIST, true);
        Serial.println(F("[AUTO-HUM] Air Assist OFF phase complete - ON burst"));
      }
    }
  } else {
    // Above assist floor - stop air assist if running
    if (g_airAssistBurstActive) {
      g_airAssistBurstActive = false;
      g_airAssistInOnPhase = false;
      relayManager_setRelay(RELAY_AIR_ASSIST, false);
      Serial.println(F("[AUTO-HUM] Above Assist floor - stopping Air Assist"));
    }
  }
}

// ============================================
// CO2 Automation Loop (GH-CO2)
// ============================================

void automation_runCO2Loop() {
  // Skip if calibration is active
  if (g_systemState.calibrationActive) {
    return;
  }

  // Skip if CO2 override is active (manual control from web UI)
  if (automation_isCO2OverrideActive()) {
    return;
  }

  // Get current CO2
  uint16_t currentCO2 = sensors_isCO2Valid() ?
                        g_systemState.currentCO2 :
                        g_systemState.lastKnownGoodCO2;

  // --- HIGH LIMIT CHECK: Above high limit, turn ON exhaust fan ---
  // GH-CO2-002
  if (currentCO2 > g_thresholds.co2HighLimit) {
    if (!relayManager_isRelayOn(RELAY_EXHAUST)) {
      Serial.print(F("[AUTO-CO2] CO2 high ("));
      Serial.print(currentCO2);
      Serial.print(F(" > "));
      Serial.print(g_thresholds.co2HighLimit);
      Serial.println(F(") - turning ON exhaust fan"));
      relayManager_setRelay(RELAY_EXHAUST, true);
    }
  }

  // --- LOW TARGET CHECK: At or below low target, turn OFF exhaust fan ---
  if (currentCO2 <= g_thresholds.co2LowTarget) {
    if (relayManager_isRelayOn(RELAY_EXHAUST)) {
      Serial.print(F("[AUTO-CO2] CO2 at target ("));
      Serial.print(currentCO2);
      Serial.print(F(" <= "));
      Serial.print(g_thresholds.co2LowTarget);
      Serial.println(F(") - turning OFF exhaust fan"));
      relayManager_setRelay(RELAY_EXHAUST, false);
    }
  }

  // --- EMERGENCY THRESHOLD: Push alert ---
  // GH-CO2-003
  static bool emergencyAlertSent = false;
  if (currentCO2 >= g_thresholds.co2Emergency) {
    if (!emergencyAlertSent) {
      emergencyAlertSent = true;
      Serial.print(F("[AUTO-CO2] EMERGENCY! CO2 at "));
      Serial.print(currentCO2);
      Serial.print(F(" ppm (threshold: "));
      Serial.print(g_thresholds.co2Emergency);
      Serial.println(F(")"));
      network_sendAlert("CO2 Emergency",
                       ("CO2 level: " + String(currentCO2) + " ppm. Check ventilation!").c_str());
    }
  } else {
    // Reset emergency flag once CO2 drops 50 ppm below emergency threshold
    if (currentCO2 < g_thresholds.co2Emergency - 50) {
      emergencyAlertSent = false;
    }
  }
}

// ============================================
// Night Mode Operation (GH-NM)
// ============================================

void automation_checkNightMode() {
  bool nightMode = rtc_isNightMode();

  if (nightMode != g_systemState.nightModeActive) {
    g_systemState.nightModeActive = nightMode;

    if (nightMode) {
      Serial.println(F("[AUTO-NM] Night Mode ACTIVATED (21:00-10:00)"));
      Serial.println(F("[AUTO-NM]   Locking out: Compressor + Air Assist"));

      // GH-NM-002: Force OFF compressor and air assist
      if (relayManager_isRelayOn(RELAY_COMPRESSOR)) {
        relayManager_setRelay(RELAY_COMPRESSOR, false);
        Serial.println(F("[AUTO-NM]   Compressor forced OFF"));
      }
      if (relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
        relayManager_setRelay(RELAY_AIR_ASSIST, false);
        g_airAssistBurstActive = false;
        g_airAssistInOnPhase = false;
        Serial.println(F("[AUTO-NM]   Air Assist forced OFF"));
      }
    } else {
      Serial.println(F("[AUTO-NM] Night Mode DEACTIVATED - all systems available"));
    }
  }

  // Continuous enforcement during night mode
  if (g_systemState.nightModeActive) {
    if (relayManager_isRelayOn(RELAY_COMPRESSOR)) {
      Serial.println(F("[AUTO-NM] Night mode - forcing compressor OFF"));
      relayManager_setRelay(RELAY_COMPRESSOR, false);
    }
    if (relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
      Serial.println(F("[AUTO-NM] Night mode - forcing Air Assist OFF"));
      relayManager_setRelay(RELAY_AIR_ASSIST, false);
      g_airAssistBurstActive = false;
      g_airAssistInOnPhase = false;
    }
  }
}

// ============================================
// Temperature Alert Monitoring (GH-TEMP-002)
// ============================================

void automation_checkTemperatureAlerts() {
  float currentTemp = sensors_isTemperatureValid() ?
                      g_systemState.currentTemp :
                      g_systemState.lastKnownGoodTemp;

  static bool highAlertSent = false;
  static bool lowAlertSent = false;

  // High temperature alert
  if (currentTemp >= TEMP_ALERT_HIGH_C) {
    if (!highAlertSent) {
      highAlertSent = true;
      Serial.print(F("[AUTO-TEMP] HIGH TEMPERATURE ALERT: "));
      Serial.print(currentTemp, 1);
      Serial.println(F(" C"));
      network_sendAlert("High Temperature Alert",
                       ("Temperature: " + String(currentTemp, 1) + " C").c_str());
    }
  } else if (currentTemp < TEMP_ALERT_HIGH_C - 1.0f) {
    highAlertSent = false;
  }

  // Low temperature alert
  if (currentTemp <= TEMP_ALERT_LOW_C) {
    if (!lowAlertSent) {
      lowAlertSent = true;
      Serial.print(F("[AUTO-TEMP] LOW TEMPERATURE ALERT: "));
      Serial.print(currentTemp, 1);
      Serial.println(F(" C"));
      network_sendAlert("Low Temperature Alert",
                       ("Temperature: " + String(currentTemp, 1) + " C").c_str());
    }
  } else if (currentTemp > TEMP_ALERT_LOW_C + 1.0f) {
    lowAlertSent = false;
  }
}

// ============================================
// Threshold Management
// ============================================

AutomationThresholds* automation_getThresholds() {
  return &g_thresholds;
}

void automation_updateThresholds(const AutomationThresholds* newThresholds) {
  // Validate individual ranges
  if (newThresholds->humHoHFloor < 0 || newThresholds->humHoHFloor > 100) {
    Serial.println(F("[AUTO] Invalid HOH floor - rejecting"));
    return;
  }
  if (newThresholds->humAssistFloor < 0 || newThresholds->humAssistFloor > 100) {
    Serial.println(F("[AUTO] Invalid Assist floor - rejecting"));
    return;
  }
  if (newThresholds->humCeiling < 0 || newThresholds->humCeiling > 100) {
    Serial.println(F("[AUTO] Invalid Ceiling - rejecting"));
    return;
  }
  if (newThresholds->co2HighLimit < 400 || newThresholds->co2HighLimit > 5000) {
    Serial.println(F("[AUTO] Invalid CO2 high limit - rejecting"));
    return;
  }
  if (newThresholds->co2LowTarget < 400 || newThresholds->co2LowTarget > 5000) {
    Serial.println(F("[AUTO] Invalid CO2 low target - rejecting"));
    return;
  }
  if (newThresholds->co2Emergency < 400 || newThresholds->co2Emergency > 5000) {
    Serial.println(F("[AUTO] Invalid CO2 emergency - rejecting"));
    return;
  }

  // Validate relationships between thresholds
  if (newThresholds->humAssistFloor > newThresholds->humHoHFloor) {
    Serial.println(F("[AUTO] Assist floor must be <= HOH floor - rejecting"));
    return;
  }
  if (newThresholds->humHoHFloor >= newThresholds->humCeiling) {
    Serial.println(F("[AUTO] HOH floor must be < Ceiling - rejecting"));
    return;
  }
  if (newThresholds->co2LowTarget >= newThresholds->co2HighLimit) {
    Serial.println(F("[AUTO] CO2 low target must be < high limit - rejecting"));
    return;
  }
  if (newThresholds->co2HighLimit >= newThresholds->co2Emergency) {
    Serial.println(F("[AUTO] CO2 high limit must be < emergency - rejecting"));
    return;
  }
  if (newThresholds->assistOnSec > newThresholds->assistOffSec) {
    Serial.println(F("[AUTO] Assist ON time exceeds OFF time - rejecting"));
    return;
  }

  memcpy(&g_thresholds, newThresholds, sizeof(AutomationThresholds));

  Serial.println(F("[AUTO] Thresholds updated and validated:"));
  Serial.print(F("  HOH Floor: ")); Serial.println(g_thresholds.humHoHFloor);
  Serial.print(F("  Assist Floor: ")); Serial.println(g_thresholds.humAssistFloor);
  Serial.print(F("  Ceiling: ")); Serial.println(g_thresholds.humCeiling);
  Serial.print(F("  CO2 High: ")); Serial.println(g_thresholds.co2HighLimit);
  Serial.print(F("  CO2 Low: ")); Serial.println(g_thresholds.co2LowTarget);
  Serial.print(F("  CO2 Emergency: ")); Serial.println(g_thresholds.co2Emergency);
}

bool automation_isAirAssistBurstActive() {
  return g_airAssistBurstActive;
}

// ============================================
// Manual Override (Per-Subsystem)
// ============================================

void automation_activateHumidityOverride() {
  g_humidityOverrideActive = true;
  g_humidityOverrideStart = millis();
  Serial.println(F("[AUTO] Humidity override ACTIVATED - 10 min"));
}

void automation_activateCO2Override() {
  g_co2OverrideActive = true;
  g_co2OverrideStart = millis();
  Serial.println(F("[AUTO] CO2 override ACTIVATED - 10 min"));
}

void automation_deactivateHumidityOverride() {
  g_humidityOverrideActive = false;
  Serial.println(F("[AUTO] Humidity override DEACTIVATED"));
}

void automation_deactivateCO2Override() {
  g_co2OverrideActive = false;
  Serial.println(F("[AUTO] CO2 override DEACTIVATED"));
}

void automation_deactivateAllOverrides() {
  g_humidityOverrideActive = false;
  g_co2OverrideActive = false;
  Serial.println(F("[AUTO] All overrides DEACTIVATED"));
}

bool automation_isHumidityOverrideActive() {
  if (g_humidityOverrideActive && (millis() - g_humidityOverrideStart >= MANUAL_OVERRIDE_TIMEOUT_SEC * 1000UL)) {
    g_humidityOverrideActive = false;
    Serial.println(F("[AUTO] Humidity override TIMEOUT - automation resumed"));
  }
  return g_humidityOverrideActive;
}

bool automation_isCO2OverrideActive() {
  if (g_co2OverrideActive && (millis() - g_co2OverrideStart >= MANUAL_OVERRIDE_TIMEOUT_SEC * 1000UL)) {
    g_co2OverrideActive = false;
    Serial.println(F("[AUTO] CO2 override TIMEOUT - automation resumed"));
  }
  return g_co2OverrideActive;
}

unsigned long automation_getHumidityOverrideRemaining() {
  if (!g_humidityOverrideActive) return 0;
  unsigned long elapsed = millis() - g_humidityOverrideStart;
  if (elapsed >= MANUAL_OVERRIDE_TIMEOUT_SEC * 1000UL) return 0;
  return (MANUAL_OVERRIDE_TIMEOUT_SEC * 1000UL) - elapsed;
}

unsigned long automation_getCO2OverrideRemaining() {
  if (!g_co2OverrideActive) return 0;
  unsigned long elapsed = millis() - g_co2OverrideStart;
  if (elapsed >= MANUAL_OVERRIDE_TIMEOUT_SEC * 1000UL) return 0;
  return (MANUAL_OVERRIDE_TIMEOUT_SEC * 1000UL) - elapsed;
}
