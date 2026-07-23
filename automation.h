/*
   automation.h
   GrowHub32 - Environmental Automation Logic
   Version: 1.2.3

   Handles:
   - Humidity control loop (GH-HUM)
   - CO2 control loop (GH-CO2)
   - Night Mode enforcement (GH-NM)
   - Temperature alert monitoring (GH-TEMP-002)
   - Per-subsystem manual override with auto-expiry
*/

#ifndef AUTOMATION_H
#define AUTOMATION_H

#include <Arduino.h>
#include "config.h"
#include "pid_controller.h"

// Runtime-adjustable thresholds (modifiable via Web UI)
// Relationship requirements enforced by automation_updateThresholds():
//   humAssistFloor <= humHoHFloor < humCeiling
//   co2LowTarget < co2HighLimit < co2Emergency
struct AutomationThresholds {
  float humHoHFloor;
  float humAssistFloor;
  float humCeiling;
  uint16_t assistOnSec;
  uint16_t assistOffSec;
  uint16_t co2HighLimit;
  uint16_t co2LowTarget;
  uint16_t co2Emergency;
};

// Public API
void automation_init();
void automation_loadDefaults();
void automation_runHumidityLoop();
void automation_runCO2Loop();
void automation_checkNightMode();
void automation_checkTemperatureAlerts();

// Threshold access (for Web UI)
AutomationThresholds* automation_getThresholds();
void automation_updateThresholds(const AutomationThresholds* newThresholds);

// Air Assist burst timing state (exposed for logging)
bool automation_isAirAssistBurstActive();

// PID controller access (for adaptive.cpp calibration)
PIDController* automation_getPIDController();

// Manual override per subsystem (scoped — humidity and CO2 are independent)
void automation_activateHumidityOverride();
void automation_activateCO2Override();
void automation_deactivateHumidityOverride();
void automation_deactivateCO2Override();
void automation_deactivateAllOverrides();
bool automation_isHumidityOverrideActive();
bool automation_isCO2OverrideActive();
unsigned long automation_getHumidityOverrideRemaining();
unsigned long automation_getCO2OverrideRemaining();

// Manual override per subsystem (scoped — humidity and CO2 are independent)
void automation_activateHumidityOverride();
void automation_activateCO2Override();
void automation_deactivateHumidityOverride();
void automation_deactivateCO2Override();
void automation_deactivateAllOverrides();
bool automation_isHumidityOverrideActive();
bool automation_isCO2OverrideActive();
unsigned long automation_getHumidityOverrideRemaining();
unsigned long automation_getCO2OverrideRemaining();

#endif // AUTOMATION_H
