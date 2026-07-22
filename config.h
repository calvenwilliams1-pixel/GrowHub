/**
 * @file config.h
 * @brief GrowHub32 v1.3 Unified Architecture — Central Configuration
 * 
 * Single source of truth for all hardware pins, timing constants,
 * thresholds, and system parameters. Every magic number in the
 * codebase must trace back to a define in this file.
 * 
 * v1.3 changes:
 *   - Fixed relay mapping (Exhaust/Compressor replacing Heater/Lights)
 *   - Added PID controller constants
 *   - Added active calibration v1.3 constants
 *   - Added night mode schedule defines
 *   - Centralized I2C addresses
 *   - Added SD card SPI pin definitions
 *   - Added Web UI / WebSocket port definitions
 *   - Added default humidity setpoint
 *   - Renamed safety guardrails to SAFETY_*_HARD pattern (no redefinition)
 *   - Deprecated CALIBRATION_DURATION_SEC in favor of CALIBRATION_TOTAL_SEC
 *   - Added pre-calculated MANUAL_OVERRIDE_TIMEOUT_MS
 *   - Lowered DEFAULT_HUM_CEILING to 88% (below SAFETY_HUM_CEILING_HARD 90%)
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// SYSTEM
// ============================================================
#define FIRMWARE_VERSION             "1.3.0"
#define DEVICE_NAME                  "GrowHub32"

// ============================================================
// HARDWARE: Relay Mapping (Active LOW)
// ============================================================
// v1.3: HEATER/LIGHTS replaced with EXHAUST/COMPRESSOR
// to match relay_manager.h and actual hardware wiring
#define RELAY_HOH                       0       // Humidifier
#define RELAY_AIR_ASSIST                1       // Air Assist Valve
#define RELAY_EXHAUST                   2       // Exhaust Fan (was RELAY_HEATER)
#define RELAY_COMPRESSOR                3       // Air Compressor (was RELAY_LIGHTS)
#define RELAY_COUNT                     4

// ============================================================
// HARDWARE: I2C Addresses
// ============================================================
#define SCD40_I2C_ADDR                  0x62    // Sensirion SCD40/SCD41
#define DS3231_I2C_ADDR                 0x68    // Maxim DS3231 RTC

// ============================================================
// HARDWARE: I2C Bus Pins
// ============================================================
#define I2C_SDA_PIN                     21      // ESP32 default I2C SDA
#define I2C_SCL_PIN                     22      // ESP32 default I2C SCL

// ============================================================
// HARDWARE: SD Card (SPI)
// ============================================================
#define SD_CS_PIN                       5
#define SD_SCK_PIN                      18
#define SD_MISO_PIN                     19
#define SD_MOSI_PIN                     23
#define SD_CARD_DETECT_PIN              -1      // Not connected (reserved for future use)

// ============================================================
// SENSORS: SCD40
// ============================================================
// SCD40 produces new data every ~5 seconds in periodic mode
#define SCD40_MEASURE_INTERVAL_MS       5000

// SENSOR_POLL_INTERVAL_MS is the CHECK interval, not the read interval.
// sensors_poll() gates on SCD40 data-ready status before reading.
// 2-second checks yield worst-case 2s latency from data-ready to detection.
// Must be <= SCD40_MEASURE_INTERVAL_MS to avoid missing data.
#define SENSOR_POLL_INTERVAL_MS         2000
#define SENSOR_RETRY_INTERVAL_MS        5000
#define SENSOR_FAULT_TIMEOUT_MS         10000UL // 10 seconds without valid data = fault
#define SCD40_TEMP_OFFSET               0.0f
#define SCD40_ALTITUDE                  0

// ============================================================
// TIMING: Logging
// ============================================================
#define LOG_INTERVAL_MS                 60000UL // Log to SD card every 60 seconds

// ============================================================
// NIGHT MODE (Noise Ordinance)
// ============================================================
// Single source of truth for night mode schedule.
// rtc_handler.cpp uses these; do not hardcode hours elsewhere.
// Night mode: NIGHT_MODE_START_HOUR:00 to NIGHT_MODE_END_HOUR:00 next day
// Spans midnight (e.g., 21:00 → 10:00 next day)
#define NIGHT_MODE_START_HOUR           21      // 9:00 PM
#define NIGHT_MODE_END_HOUR             10      // 10:00 AM next day

// ============================================================
// TEMPERATURE: Alert Thresholds
// ============================================================
#define TEMP_ALERT_HIGH_C               32.0f
#define TEMP_ALERT_LOW_C                12.0f

// ============================================================
// TEMPERATURE BANDS (Adaptive Learning)
// ============================================================
#define BAND_18_21_LOW                  18.0f
#define BAND_18_21_HIGH                 21.0f
#define BAND_21_24_LOW                  21.0f
#define BAND_21_24_HIGH                 24.0f
#define BAND_24_27_LOW                  24.0f
#define BAND_24_27_HIGH                 27.0f
#define BAND_27_30_LOW                  27.0f
#define BAND_27_30_HIGH                 30.0f

#define NUM_TEMP_BANDS                  4

// SD card profile filenames per band
#define BAND_18_21_FILE                 "/profiles/band_18_21.json"
#define BAND_21_24_FILE                 "/profiles/band_21_24.json"
#define BAND_24_27_FILE                 "/profiles/band_24_27.json"
#define BAND_27_30_FILE                 "/profiles/band_27_30.json"

#define PROFILE_JSON_MAX_SIZE           1024

// ============================================================
// ADAPTIVE LEARNING (EMA & Confidence)
// ============================================================
#define DEFAULT_EMA_WEIGHT              0.30f
#define EMA_WEIGHT_MIN                  0.10f
#define EMA_WEIGHT_MAX                  0.50f
#define CONFIDENCE_MAX                  0.90f

// ============================================================
// ACTIVE CALIBRATION v1.3
// ============================================================
#define CALIBRATION_TOTAL_SEC           1200    // Max total calibration duration (20 min)
#define CALIBRATION_STABILIZE_SEC       300     // Phase 1: let environment settle (5 min)
#define CALIBRATION_RISE_MAX_SEC        600     // Phase 2: max time for rise phase (10 min)
#define CALIBRATION_PLATEAU_SLOPE_PCT   0.5f    // % RH per minute — below this = plateau
#define CALIBRATION_PLATEAU_WINDOW_SEC  120     // First slope evaluation window (2 min)
#define CALIBRATION_PLATEAU_CONFIRM_SEC 90      // Second window to confirm plateau (90 sec)
#define CALIBRATION_MIN_EXPECTED_RISE_PCT  3.0f // Min RH rise to validate calibration
#define CALIBRATION_MIN_RH_DELTA        2.0f    // Min total RH swing for valid calibration

// @deprecated — Backward compatibility alias. Use CALIBRATION_TOTAL_SEC. Remove in v1.4.
#define CALIBRATION_DURATION_SEC        CALIBRATION_TOTAL_SEC

// ============================================================
// PID CONTROLLER v1.3
// ============================================================
#define PID_AUTO_ENABLE_CONFIDENCE      0.70f
#define PID_CYCLE_MS                    30000UL
#define PID_MIN_RELAY_ON_MS             3000UL
#define PID_MIN_RELAY_OFF_MS            3000UL
#define PID_DUTY_CLAMP_LOW_PCT          5.0f
#define PID_DUTY_CLAMP_HIGH_PCT         95.0f
#define PID_BOOST_THRESHOLD_PCT         5.0f
#define PID_SAMPLE_TIME_MS              2000UL
#define PID_DEFAULT_KP                  0.5f
#define PID_DEFAULT_KI                  0.01f
#define PID_DEFAULT_KD                  0.1f

// ============================================================
// AUTOMATION: Default Thresholds (Bang-Bang Fallback)
// ============================================================
// Hierarchy: HOH Floor (80%) → Assist Floor (70%) → Ceiling (88%)
// Safety hard ceiling is at 90% (SAFETY_HUM_CEILING_HARD)
#define DEFAULT_HUM_HOH_FLOOR           80.0f   // HOH turns ON below this
#define DEFAULT_HUM_ASSIST_FLOOR        70.0f   // Air Assist joins below this (critical)
#define DEFAULT_HUM_CEILING             88.0f   // Everything OFF at or above this (< safety hard 90%)
#define DEFAULT_HUM_ASSIST_ON_SEC       3       // Air Assist burst ON duration
#define DEFAULT_HUM_ASSIST_OFF_SEC      10      // Air Assist burst OFF duration
#define DEFAULT_HUMIDITY_SETPOINT       87.5f   // PID target humidity

// ============================================================
// AUTOMATION: Default CO2 Thresholds
// ============================================================
#define DEFAULT_CO2_HIGH_LIMIT          800
#define DEFAULT_CO2_LOW_TARGET          600
#define DEFAULT_CO2_EMERGENCY           1200

// ============================================================
// SAFETY: Override & Recovery
// ============================================================
// Pre-calculated millisecond value (avoids caller overflow bugs)
#define MANUAL_OVERRIDE_TIMEOUT_MS      (600UL * 1000UL)  // 10 minutes
// @deprecated — Use MANUAL_OVERRIDE_TIMEOUT_MS. Remove in v1.4.
#define MANUAL_OVERRIDE_TIMEOUT_SEC     600UL

// Default recovery rate: ~1% RH per minute = 0.0167% RH per second
#define DEFAULT_RECOVERY_RATE_PCT_PER_SEC  0.0167f

// Humidifier runtime watchdog limits HOH ON time per 2-hour window.
// These are for the humidifier runtime watchdog (g_watchdog in automation.cpp),
// NOT the ESP32 hardware task watchdog (esp_task_wdt in safety.cpp).
#define WATCHDOG_HOH_MAX_ON_MS          1800000UL // 30 minutes
#define WATCHDOG_HOH_WINDOW_MS          7200000UL // 2 hours

// ============================================================
// SAFETY: Absolute Guardrails (Hard Limits)
// ============================================================
// These are NEVER exceeded by any control mode.
// Automation thresholds (above) can be adjusted at runtime; these cannot.
// SAFETY_HUM_CEILING_HARD must be > DEFAULT_HUM_CEILING (88%).
#define SAFETY_HUM_FLOOR_HARD           40.0f
#define SAFETY_HUM_CEILING_HARD         90.0f   // Emergency cutoff, calibration abort
#define SAFETY_TEMP_FLOOR_HARD          15.0f
#define SAFETY_TEMP_CEILING_HARD        35.0f

// ============================================================
// WEB UI & WEBSOCKET
// ============================================================
#define WEB_SERVER_PORT                 80
#define WEBSOCKET_PORT                  81
#define WEBSOCKET_UPDATE_INTERVAL_MS    1000UL

// ============================================================
// SD LOGGER
// ============================================================
#define SUMMARY_ARCHIVE_FILE            "/logs/daily_summary.csv"
#define LOG_RETENTION_DAYS              30

// ============================================================
// NETWORK
// ============================================================
#define WIFI_RETRY_INTERVAL_MS          30000UL
#define WIFI_MAX_RETRIES                5

#endif // CONFIG_H
