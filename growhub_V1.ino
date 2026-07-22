/*
   GrowHub32_MainNode.ino
   Main Automation Node - ESP32 30-Pin Dev Board
   Version: 1.3.0
   Author: Calvin

   Hardware:
   - ESP32 30-Pin Dev Board (CP2102 Type-C)
   - SCD40 CO2/Temp/RH Sensor (I2C: 0x62)
   - DS3231 RTC Module (I2C: 0x68)
   - 4-Channel Relay Board (Active LOW, JD-VCC jumper for external 5V)
   - TF MicroSD SPI Module

   v1.3 Revision:
   - Unified automation architecture (PID + bang-bang hybrid)
   - SystemState struct extracted to system_state.h
   - Added PID controller state fields to initializer
   - Fixed boot order: adaptive_loadProfiles() after adaptive_init()
   - Active calibration v1.3 with phase state machine
   - Night mode calibration scheduling via rtc_minutesUntilNightMode()
   - All g_systemState writes protected by g_stateMux
   - Heap-free alert paths (snprintf stack buffers)
   - Watchdog cooldown recovery for humidifier runtime watchdog
   - checkSensorFaults() simplified to alerting only (sensors.cpp owns fault flags)
*/

#include <Arduino.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "system_state.h"
#include "sensors.h"
#include "rtc_handler.h"
#include "relay_manager.h"
#include "automation.h"
#include "sd_logger.h"
#include "network.h"
#include "web_ui.h"
#include "adaptive.h"
#include "safety.h"

// ============================================================
// External Declarations
// ============================================================

extern void network_sendAlert(const char* title, const char* message);

// ============================================================
// Global System State
// ============================================================

SystemState g_systemState = {
  .currentTemp = 0.0f,
  .currentHumidity = 0.0f,
  .currentCO2 = 0,
  .lastKnownGoodTemp = 22.0f,
  .lastKnownGoodHumidity = 85.0f,
  .lastKnownGoodCO2 = 450,
  .tempSensorFault = false,
  .humiditySensorFault = false,
  .co2SensorFault = false,
  .sensorFaultTimer = 0,
  .nightModeActive = false,
  .calibrationActive = false,
  .hoHActive = false,
  .airAssistActive = false,
  .exhaustFanActive = false,
  .compressorActive = false,
  .wifiConnected = false,
  .apModeActive = false,
  .lastNTFYAlert = 0,
  .fridgeHeartbeatLost = false,
  .fridgeLastSequence = 0,
  .fridgeTemp = 4.0f,
  .currentDutyCycle = 0.0f,
  .pidOutput = 0.0f,
  .setpointHumidity = 87.5f,
  .currentBand = 0
};

// ============================================================
// Timing Variables
// ============================================================

static unsigned long g_lastSensorPoll = 0;
static unsigned long g_lastLogWrite = 0;
static unsigned long g_lastESPNOWCheck = 0;
static unsigned long g_lastWiFiCheck = 0;
static unsigned long g_lastSDCacheSave = 0;

// ============================================================
// Forward Declarations
// ============================================================

void checkSensorFaults(unsigned long now);

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println(F(""));
  Serial.println(F("============================================"));
  Serial.println(F("       GrowHub32 Main Node v1.3.0"));
  Serial.println(F("  Environmental Automation Controller"));
  Serial.println(F("============================================"));
  Serial.println(F(""));

  // Step 1: Relay Safety Initialization
  // MUST be first to prevent floating GPIOs from triggering appliances during boot
  Serial.println(F("[BOOT] Step 1: Relay safety init..."));
  relayManager_init();

  // Step 2: Hardware Watchdog Timer
  // Must be early so all subsequent blocking operations are protected
  Serial.println(F("[BOOT] Step 2: Watchdog init..."));
  safety_initWatchdog();
  safety_feedWatchdog();

  // Step 3: I2C Bus Initialization
  Serial.println(F("[BOOT] Step 3: I2C bus init..."));
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  safety_feedWatchdog();

  // Step 4: Real-Time Clock
  Serial.println(F("[BOOT] Step 4: RTC init..."));
  if (!rtc_init()) {
    Serial.println(F("[BOOT] WARNING: RTC init FAILED - time-dependent features degraded"));
  }
  safety_feedWatchdog();

  // Step 5: SCD40 Sensor Initialization
  Serial.println(F("[BOOT] Step 5: SCD40 sensor init..."));
  if (!sensors_init()) {
    Serial.println(F("[BOOT] WARNING: SCD40 init FAILED - sensor unavailable"));
    g_systemState.tempSensorFault = true;
    g_systemState.humiditySensorFault = true;
    g_systemState.co2SensorFault = true;
  }
  safety_feedWatchdog();

  // Step 6: SD Card & Logging
  Serial.println(F("[BOOT] Step 6: SD card init..."));
  safety_feedWatchdog();
  if (!sdLogger_init()) {
    Serial.println(F("[BOOT] WARNING: SD card init FAILED - logging and profiles disabled"));
  }
  safety_feedWatchdog();

  // Step 7: Network & Alerts
  Serial.println(F("[BOOT] Step 7: Network init..."));
  safety_feedWatchdog();
  network_init();

  // Deferred watchdog failure alert (can only send after WiFi is up)
  if (safety_didWatchdogInitFail()) {
    network_sendAlert("Watchdog Init Failed",
                     "Task watchdog initialization returned non-OK. System may not auto-reboot on hang.");
  }

  // Step 7b: OTA Update Service
  // Available in both station and AP mode for recovery flexibility.
  // Relays are force-killed and active calibration is aborted before flash write.
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }
  ArduinoOTA.setPort(OTA_PORT);

  ArduinoOTA.onStart([]() {
    Serial.println(F("[OTA] Update starting - forcing safe state..."));

    // Abort active calibration to prevent orphaned profile state across reboot
    if (adaptive_isCalibrating()) {
      Serial.println(F("[OTA] WARNING: Aborting active calibration before update"));
      adaptive_cancelCalibration();
    }

    // De-energize all relays before flash write blocks the main loop.
    // Prevents uncontrolled actuator runtime during OTA transfer.
    relayManager_forceAllOff();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("[OTA] Update complete - rebooting"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)      Serial.println(F("Auth Failed"));
    else if (error == OTA_BEGIN_ERROR) Serial.println(F("Begin Failed"));
    else if (error == OTA_CONNECT_ERROR) Serial.println(F("Connect Failed"));
    else if (error == OTA_RECEIVE_ERROR) Serial.println(F("Receive Failed"));
    else if (error == OTA_END_ERROR)   Serial.println(F("End Failed"));
  });

  ArduinoOTA.begin();
  Serial.print(F("[OTA] Service started - upload via IDE or espota.py on port "));
  Serial.println(OTA_PORT);

  // Step 8: Web User Interface
  Serial.println(F("[BOOT] Step 8: Web UI init..."));
  webUI_init();

  // Step 9: Automation Engine (v1.3 - fixed boot order)
  Serial.println(F("[BOOT] Step 9: Automation engine init..."));
  automation_init();
  adaptive_init();
  adaptive_loadProfiles();  // Must be after adaptive_init()

  // Step 10: System Ready
  safety_feedWatchdog();
  Serial.println(F(""));
  Serial.println(F("============================================"));
  Serial.println(F("       System Initialization Complete"));
  Serial.println(F("       http://growhub.local"));
  Serial.print(F("       IP: "));
  Serial.println(network_getIPAddress());
  Serial.print(F("       RTC: "));
  char timeStr[24];
  rtc_getTimeString(timeStr, sizeof(timeStr));
  Serial.println(timeStr);
  Serial.print(F("       OTA: port "));
  Serial.println(OTA_PORT);
  Serial.println(F("============================================"));
  Serial.println(F(""));
  network_sendAlert("GrowHub32 Online", "Main Node v1.3.0 booted successfully.");
  sdLogger_logSystemEvent("System boot complete - v1.3.0");

  g_lastSensorPoll = millis();
  g_lastLogWrite = millis();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  unsigned long now = millis();

  // Feed watchdog every loop iteration
  safety_feedWatchdog();

  // Handle OTA updates (non-blocking when idle)
  ArduinoOTA.handle();

  // Handle RTC serial commands
  rtc_checkSerialCommand();

  // Sensor polling every 2 seconds
  if (now - g_lastSensorPoll >= SENSOR_POLL_INTERVAL_MS) {
    g_lastSensorPoll = now;
    sensors_poll();
    checkSensorFaults(now);
  }

  // Night mode check every loop
  automation_checkNightMode();

  // Temperature alerts every loop
  automation_checkTemperatureAlerts();

  // Automation logic every loop
  automation_runHumidityLoop();
  automation_runCO2Loop();

  // Safety checks every loop
  safety_checkDryRun(now);
  safety_checkFanStall(now);

  // Enforce compressor max ON time (GH-SAFE-002)
  {
    unsigned long compressorOnDuration = relayManager_getOnDuration(RELAY_COMPRESSOR);
    if (compressorOnDuration >= COMPRESSOR_MAX_ON_MS) {
      Serial.println(F("[SAFETY] Compressor max ON time reached - forcing OFF"));
      relayManager_setRelay(RELAY_COMPRESSOR, false);
    }
  }

  // Adaptive learning update (calibration state machine)
  adaptive_updateCalibration();

  // Network health check every 10 seconds
  if (now - g_lastWiFiCheck >= 10000) {
    g_lastWiFiCheck = now;
    network_checkConnection();
  }

  // ESP-NOW heartbeat check every 5 seconds
  if (now - g_lastESPNOWCheck >= 5000) {
    g_lastESPNOWCheck = now;
    network_checkFridgeHeartbeat();
  }

  // SD card logging every 60 seconds
  if (now - g_lastLogWrite >= LOG_INTERVAL_MS) {
    g_lastLogWrite = now;
    sdLogger_writeData();
  }

  // Runtime cache save every 10 minutes
  if (now - g_lastSDCacheSave >= 600000UL) {
    g_lastSDCacheSave = now;

    g_runtimeCache.totalRuntimeHours += (10.0f / 60.0f);
    g_runtimeCache.lastActiveBand = adaptive_getCurrentBand();
    AutomationThresholds* thresholds = automation_getThresholds();
    memcpy(&g_runtimeCache.thresholds, thresholds, sizeof(AutomationThresholds));
    g_runtimeCache.emaWeight = adaptive_getEMAWeight();

    sdLogger_saveCache();
  }

  // Web UI updates
  webUI_pushUpdates();
  webUI_handleClient();

  // Yield to FreeRTOS
  delay(10);
}

// ============================================================
// SENSOR FAULT ALERTING
// ============================================================
// Fault detection and flagging is owned by sensors.cpp.
// This function only handles alerting — it reads the fault flags
// that sensors.cpp sets under g_stateMux and sends notifications
// on state transitions. It does NOT write to g_systemState.

void checkSensorFaults(unsigned long now) {
  static bool sensorFaultAlertSent = false;

  // Read fault flags set by sensors.cpp
  bool anyFault = g_systemState.tempSensorFault ||
                  g_systemState.humiditySensorFault ||
                  g_systemState.co2SensorFault;

  if (anyFault && !sensorFaultAlertSent) {
    sensorFaultAlertSent = true;
    network_sendAlert("Sensor Fault", "One or more sensors have failed. System running on Last Known Good values.");
  } else if (!anyFault && sensorFaultAlertSent) {
    sensorFaultAlertSent = false;
    network_sendAlert("Sensor Recovered", "All sensors are now responding normally.");
  }
}
