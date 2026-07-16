/*
   config.h
   GrowHub32 - Global Configuration & Constants
   Version: 1.2.5
   Revision: Added OTA update constants.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ============================================
// ESP32 PIN ASSIGNMENTS (30-Pin Dev Board)
// ============================================

// I2C Bus (Shared: SCD40 + DS3231)
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

// 4-Channel Relay Board (Active LOW)
#define RELAY_HOH_PIN       13    // IN1 - Hand-Of-God Humidifier
#define RELAY_AIR_ASSIST_PIN 12   // IN2 - Air Assist Solenoid Valve (NEEDS 10k PULL-DOWN TO GND)
#define RELAY_EXHAUST_PIN   14    // IN3 - Exhaust Fan
#define RELAY_COMPRESSOR_PIN 27   // IN4 - Air Compressor

// SPI - TF MicroSD Module
#define SD_CS_PIN           5
#define SD_SCK_PIN          18
#define SD_MOSI_PIN         23
#define SD_MISO_PIN         19

// ============================================
// SENSOR CONFIGURATION
// ============================================

#define SCD40_I2C_ADDRESS       0x62
#define SENSOR_POLL_INTERVAL_MS 2000     // GH-TEMP-001: 2 seconds
#define SENSOR_FAULT_TIMEOUT_MS 30000    // GH-TEMP-004: 30 seconds
#define SENSOR_LKG_TIMEOUT_MS   600000   // 10 minutes max on last-known-good
// SCD40 altitude compensation (meters above sea level).
// CO2 readings from NDIR sensors are pressure-dependent.
// Set to actual facility elevation for accurate CO2 measurement.
#define SCD40_DEFAULT_ALTITUDE_M   0

// ============================================
// RTC CONFIGURATION
// ============================================

#define DS3231_I2C_ADDRESS      0x68
#define NIGHT_MODE_START_HOUR   21      // 9:00 PM (GH-NM-001)
#define NIGHT_MODE_END_HOUR     10      // 10:00 AM

// ============================================
// AUTOMATION DEFAULTS (GH-HUM, GH-CO2)
// ============================================

// Humidity (user-adjustable via UI)
// Relationship must be: humAssistFloor <= humHoHFloor < humCeiling
#define DEFAULT_HUM_HOH_FLOOR       80.0f   // % RH - HOH humidifier ON below this
#define DEFAULT_HUM_ASSIST_FLOOR    75.0f   // % RH - Air Assist ON below this
#define DEFAULT_HUM_CEILING         95.0f   // % RH - Both OFF at or above this
#define DEFAULT_HUM_ASSIST_ON_SEC   5       // Air Assist ON duration per burst
#define DEFAULT_HUM_ASSIST_OFF_SEC  15      // Air Assist OFF duration between bursts

// CO2 (user-adjustable via UI)
// Relationship must be: co2LowTarget < co2HighLimit < co2Emergency
#define DEFAULT_CO2_HIGH_LIMIT      650     // ppm - Exhaust fan ON above this
#define DEFAULT_CO2_LOW_TARGET      600     // ppm - Exhaust fan OFF at or below this
#define DEFAULT_CO2_EMERGENCY       800     // ppm - Push alert (GH-CO2-003)

// Temperature Alerts (GH-TEMP-002)
#define TEMP_ALERT_HIGH_C           30.0f
#define TEMP_ALERT_LOW_C            15.0f

// ============================================
// SAFETY GUARDRAILS (GH-SAFE)
// ============================================

#define RELAY_MAX_CYCLES_PER_MIN    10      // GH-SAFE-001
#define COMPRESSOR_MAX_ON_SEC       300     // GH-SAFE-002: 5 minutes
#define COMPRESSOR_COOLDOWN_SEC     600     // GH-SAFE-002: 10 minutes

// Compressor safety timing (precomputed milliseconds to avoid repeated multiplication
// and eliminate integer promotion risks in comparisons)
#define COMPRESSOR_MAX_ON_MS       ((unsigned long)COMPRESSOR_MAX_ON_SEC * 1000UL)
#define COMPRESSOR_COOLDOWN_MS     ((unsigned long)COMPRESSOR_COOLDOWN_SEC * 1000UL)

// Relay cycle limiting window
#define RELAY_CYCLE_WINDOW_MS      60000   // 60-second rolling window for cycle counting

#define FAN_STALL_CHECK_SEC         120     // GH-SAFE-003: 2 minutes no CO2 drop
#define FAN_STALL_THRESHOLD_PPM     25      // Min CO2 decrease to confirm fan is working (accounts for SCD40 +/-50ppm accuracy)
#define HOH_DRY_RUN_CHECK_SEC       600     // GH-SAFE-004: 10 minutes no RH rise
#define DRY_RUN_THRESHOLD_PCT       1.5f    // Min RH increase to confirm humidifier is working (accounts for sensor noise)
#define WDT_TIMEOUT_SEC             60      // GH-SAFE-006: Task Watchdog

// Manual override timeout (web UI relay commands suspend automation for this duration)
#define MANUAL_OVERRIDE_TIMEOUT_SEC 600     // 10 minutes before auto-resume

// Air Assist Duty Cycle Guard (GH-HUM-004)
#define AIR_ASSIST_MAX_DUTY_PCT     50      // ON time cannot exceed 50% of cycle

// ============================================
// WIRELESS / NETWORK
// ============================================

// TODO: Migrate WiFi credentials to Preferences.h (NVS) for production use
#define WIFI_SSID                   "DSLAP"
#define WIFI_PASSWORD               "12345678910"
#define WIFI_DISCONNECT_TIMEOUT_MS  15000   // GH-NET-002: 15 seconds
#define WIFI_SCAN_INTERVAL_MS       30000   // GH-NET-002: 30 seconds (reduced from 10 minutes)
#define AP_SSID                     "GrowHub32_Backup"
#define AP_PASSWORD                 "growhub32"

// ntfy.sh Push Notifications
#define NTFY_ENDPOINT               "ntfy.sh/calvin_growhub32_alerts"
#define NTFY_MIN_INTERVAL_MS        30000   // Minimum 30s between alerts

// ESP-NOW (Remote Fridge Node)
#define ESPNOW_HEARTBEAT_INTERVAL_MS 5000  // GH-NET-003: 5 seconds
#define ESPNOW_FAIL_TIMEOUT_MS       60000  // GH-NET-004: 60 seconds no packet
#define ESPNOW_CRC16_POLY            0x8005

// Static IP Configuration
#define STATIC_IP_OCTET_1   10
#define STATIC_IP_OCTET_2   0
#define STATIC_IP_OCTET_3   0
#define STATIC_IP_OCTET_4   20

#define GATEWAY_OCTET_1     10
#define GATEWAY_OCTET_2     0
#define GATEWAY_OCTET_3     0
#define GATEWAY_OCTET_4     1

// ============================================
// OTA (Over-The-Air) Updates
// ============================================

// Allows wireless firmware uploads via Arduino IDE or espota.py.
// Leave OTA_PASSWORD empty for trusted private networks only.
// Set a password whenever multiple users share LAN access.
// Available in both station and AP mode for recovery flexibility.
#define OTA_HOSTNAME                "growhub32"
#define OTA_PASSWORD                ""
#define OTA_PORT                    3232

// ============================================
// DATA LOGGING (GH-LOG)
// ============================================

#define LOG_INTERVAL_MS             60000   // GH-LOG-001: 60 seconds
#define LOG_RETENTION_DAYS          30      // GH-LOG-002: 30 day rolling
#define SUMMARY_ARCHIVE_FILE        "/summary_archive.csv"

// ============================================
// ADAPTIVE LEARNING (GH-AL)
// ============================================

#define DEFAULT_EMA_WEIGHT          0.30f   // 30% new, 70% historical
#define EMA_WEIGHT_MIN              0.10f
#define EMA_WEIGHT_MAX              0.50f
#define CONFIDENCE_MAX              0.90f   // GH-AL-006
#define CALIBRATION_DURATION_SEC    900     // GH-AL-004: 15 minutes
#define PROFILE_JSON_MAX_SIZE       1024    // Max bytes for band profile JSON files

// Calibration thresholds
#define CALIBRATION_FALL_GATE_MS    60000UL // Minimum elapsed time before fall detection can trigger (60 seconds)
#define CALIBRATION_RISE_DELTA_PCT  1.0f    // RH must rise this far above start to confirm rising phase
#define CALIBRATION_FALL_DELTA_PCT  2.5f    // RH must drop this far below peak to confirm falling phase
#define CALIBRATION_MIN_RH_DELTA    0.5f    // Minimum RH swing required for valid calibration results

// Temperature band boundary files
#define BAND_18_21_FILE             "/profiles/band_18_21.json"
#define BAND_21_24_FILE             "/profiles/band_21_24.json"
#define BAND_24_27_FILE             "/profiles/band_24_27.json"
#define BAND_27_30_FILE             "/profiles/band_27_30.json"

// ============================================
// WEB UI
// ============================================

#define WEB_SERVER_PORT             80
#define WEBSOCKET_PORT              81
#define WEBSOCKET_UPDATE_INTERVAL_MS 1000

// ============================================
// FREERTOS CONCURRENCY GUARD
// ============================================

// Mutex for protecting g_systemState access between main loop task
// and ESP-NOW/WiFi callback tasks on dual-core ESP32
extern portMUX_TYPE g_stateMux;

// ============================================
// SYSTEM STATE STRUCTURE
// ============================================

struct SystemState {
  // Live sensor readings
  float currentTemp;
  float currentHumidity;
  uint16_t currentCO2;

  // Last known good values (for fault fallback)
  float lastKnownGoodTemp;
  float lastKnownGoodHumidity;
  uint16_t lastKnownGoodCO2;

  // Sensor fault flags
  bool tempSensorFault;
  bool humiditySensorFault;
  bool co2SensorFault;
  unsigned long sensorFaultTimer;

  // Operating modes
  bool nightModeActive;
  bool calibrationActive;

  // Relay states
  bool hoHActive;
  bool airAssistActive;
  bool exhaustFanActive;
  bool compressorActive;

  // Network state
  bool wifiConnected;
  bool apModeActive;

  // Alerting
  unsigned long lastNTFYAlert;

  // Fridge node
  bool fridgeHeartbeatLost;
  uint16_t fridgeLastSequence;
  float fridgeTemp;
};

extern SystemState g_systemState;

#endif // CONFIG_H
