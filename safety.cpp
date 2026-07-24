/*
   safety.cpp
   GrowHub32 - Hardware Protection & System Safety Implementation
   Version: 1.2.3
   Revision: Replaced hand-rolled hw_timer watchdog with ESP-IDF Task Watchdog
             Timer for guaranteed reboot on main loop hang (GH-SAFE-006).
             Watchdog failure flag is stored for deferred alerting after network init.
             No custom ISR, no hw_timer — uses the API designed for this purpose.
*/

#include "safety.h"
#include "relay_manager.h"
#include "sensors.h"
#include "system_state.h"

static DryRunState g_dryRunState = {0, 0.0f, false};
static FanStallState g_fanStallState = {0, 0, false};
static bool g_watchdogInitFailed = false;

// ============================================
// Task Watchdog Timer (GH-SAFE-006)
// Uses ESP-IDF Task Watchdog — the correct API for
// rebooting when the main loop stops feeding.
// Deinits any pre-existing TWDT from the Arduino core
// so we can apply our own WDT_TIMEOUT_SEC.
// ============================================

void safety_initWatchdog() {
  // Arduino-ESP32 core 2.x may have already initialized the TWDT
  // with a shorter default timeout. Deinit first so we can apply our own.
  esp_task_wdt_deinit();

  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_err_t initResult = esp_task_wdt_init(&wdtConfig);
  esp_err_t addResult  = esp_task_wdt_add(NULL);

  Serial.print(F("[SAFETY] Task watchdog init: "));
  Serial.print(initResult == ESP_OK ? F("OK") : F("FAILED"));
  Serial.print(F(" (code "));
  Serial.print(initResult);
  Serial.print(F("), add: "));
  Serial.print(addResult == ESP_OK ? F("OK") : F("FAILED"));
  Serial.print(F(" (code "));
  Serial.print(addResult);
  Serial.print(F("), timeout: "));
  Serial.print(WDT_TIMEOUT_SEC);
  Serial.println(F("s"));

  g_watchdogInitFailed = (initResult != ESP_OK || addResult != ESP_OK);

  if (g_watchdogInitFailed) {
    Serial.println(F("[SAFETY] WARNING: Watchdog init FAILED. Alert will fire once network is up."));
  }
}

void safety_feedWatchdog() {
  // Reset the Task Watchdog timer for the current task (Arduino loop task)
  esp_task_wdt_reset();
}

bool safety_didWatchdogInitFail() {
  return g_watchdogInitFailed;
}

// ============================================
// Humidifier Dry-Run Detection (GH-SAFE-004)
// ============================================

void safety_checkDryRun(unsigned long now) {
  if (g_systemState.hoHActive) {
    if (g_dryRunState.hoHStartTime == 0) {
      g_dryRunState.hoHStartTime = now;
      g_dryRunState.startHumidity = sensors_isHumidityValid() ?
                                    g_systemState.currentHumidity :
                                    g_systemState.lastKnownGoodHumidity;
      g_dryRunState.alertSent = false;

      Serial.print(F("[SAFETY] Dry-run monitoring started at RH: "));
      Serial.println(g_dryRunState.startHumidity, 1);
    }

    unsigned long elapsed = now - g_dryRunState.hoHStartTime;
    if (elapsed >= (HOH_DRY_RUN_CHECK_SEC * 1000UL)) {
      float currentHumidity = sensors_isHumidityValid() ?
                              g_systemState.currentHumidity :
                              g_systemState.lastKnownGoodHumidity;

      if (currentHumidity <= g_dryRunState.startHumidity + DRY_RUN_THRESHOLD_PCT) {
        if (!g_dryRunState.alertSent) {
          g_dryRunState.alertSent = true;
          Serial.println(F(""));
          Serial.println(F("============================================"));
          Serial.println(F("[SAFETY] HOH DRY-RUN DETECTED!"));
          Serial.println(F("[SAFETY] Tank may be empty or humidifier malfunctioning."));
          Serial.print(F("[SAFETY]   Started at RH: "));
          Serial.print(g_dryRunState.startHumidity, 1);
          Serial.print(F("%, Current RH: "));
          Serial.print(currentHumidity, 1);
          Serial.print(F("%  (delta: "));
          Serial.print(currentHumidity - g_dryRunState.startHumidity, 1);
          Serial.println(F("%)"));
          Serial.println(F("============================================"));
          Serial.println(F(""));

          extern void network_sendAlert(const char* title, const char* message);
          network_sendAlert("Humidifier Dry-Run Detected",
                           "HOH has run for 10+ minutes with no humidity increase. Check water tank.");
        }
      } else {
        g_dryRunState.hoHStartTime = now;
        g_dryRunState.startHumidity = currentHumidity;
        g_dryRunState.alertSent = false;
      }
    }
  } else {
    if (g_dryRunState.hoHStartTime > 0) {
      Serial.println(F("[SAFETY] Dry-run monitoring ended - HOH turned off"));
    }
    g_dryRunState.hoHStartTime = 0;
    g_dryRunState.alertSent = false;
  }
}

// ============================================
// Exhaust Fan Stall Detection (GH-SAFE-003)
// ============================================

void safety_checkFanStall(unsigned long now) {
  if (g_systemState.exhaustFanActive) {
    if (g_fanStallState.fanStartTime == 0) {
      g_fanStallState.fanStartTime = now;
      g_fanStallState.startCO2 = sensors_isCO2Valid() ?
                                 g_systemState.currentCO2 :
                                 g_systemState.lastKnownGoodCO2;
      g_fanStallState.alertSent = false;

      Serial.print(F("[SAFETY] Fan stall monitoring started at CO2: "));
      Serial.print(g_fanStallState.startCO2);
      Serial.println(F(" ppm"));
    }

    unsigned long elapsed = now - g_fanStallState.fanStartTime;
    if (elapsed >= (FAN_STALL_CHECK_SEC * 1000UL)) {
      uint16_t currentCO2 = sensors_isCO2Valid() ?
                            g_systemState.currentCO2 :
                            g_systemState.lastKnownGoodCO2;

      if (currentCO2 >= g_fanStallState.startCO2 - 5) {
        if (!g_fanStallState.alertSent) {
          g_fanStallState.alertSent = true;
          Serial.println(F(""));
          Serial.println(F("============================================"));
          Serial.println(F("[SAFETY] FAN STALL DETECTED!"));
          Serial.println(F("[SAFETY] Exhaust fan may be unplugged, blocked, or failed."));
          Serial.print(F("[SAFETY]   Started at CO2: "));
          Serial.print(g_fanStallState.startCO2);
          Serial.print(F(" ppm, Current CO2: "));
          Serial.print(currentCO2);
          Serial.print(F(" ppm  (delta: "));
          Serial.print((int16_t)currentCO2 - (int16_t)g_fanStallState.startCO2);
          Serial.println(F(" ppm)"));
          Serial.println(F("============================================"));
          Serial.println(F(""));

          extern void network_sendAlert(const char* title, const char* message);
          network_sendAlert("Fan Stall Detected",
                           "Exhaust fan has run for 2+ minutes with no CO2 reduction. Check fan.");
        }
      } else {
        g_fanStallState.fanStartTime = now;
        g_fanStallState.startCO2 = currentCO2;
        g_fanStallState.alertSent = false;
      }
    }
  } else {
    if (g_fanStallState.fanStartTime > 0) {
      Serial.println(F("[SAFETY] Fan stall monitoring ended - exhaust fan turned off"));
    }
    g_fanStallState.fanStartTime = 0;
    g_fanStallState.alertSent = false;
  }
}

bool safety_isDryRunAlertActive() {
  return g_dryRunState.alertSent;
}

bool safety_isFanStallAlertActive() {
  return g_fanStallState.alertSent;
}
