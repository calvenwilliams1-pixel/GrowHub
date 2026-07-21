/*
   config.h
   GrowHub32 - System Configuration Constants
   Version: 1.3
   Revision: Added active calibration constants, PID defaults, safety limits.
*/

#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// SYSTEM CONSTANTS
// ============================================

#define FIRMWARE_VERSION             "1.3.0"
#define DEVICE_NAME                  "GrowHub32"

// ============================================
// SENSOR CONSTANTS (SCD40)
// ============================================

#define SCD40_I2C_ADDR               0x62
#define SCD40_MEASURE_INTERVAL_MS    2000      // 2 seconds between readings
#define SCD40_TEMP_OFFSET            0.0f      // Calibration offset (degrees C)
#define SCD40_ALTITUDE               0         // Sea level (meters)

// Sensor fault detection
#define SENSOR_FAULT_TIMEOUT_MS      10000     // 10 seconds without valid data = fault
#define SENSOR_RETRY_INTERVAL_MS     5000      // 5 seconds between retries

// ============================================
// TEMPERATURE BANDS (GH-AL-001)
// ============================================

#define BAND_18_21_LOW              18.0f
#define BAND_18_21_HIGH             21.0f
#define BAND_21_24_LOW              21.0f
#define BAND_21_24_HIGH             24.0f
#define BAND_24_27_LOW              24.0f
#define BAND_24_27_HIGH             27.0f
#define BAND_27_30_LOW              27.0f
#define BAND_27_30_HIGH             30.0f

#define BAND_18_21_INDEX            0
#define BAND_21_24_INDEX            1
#define BAND_24_27_INDEX            2
#define BAND_27_30_INDEX            3

// SD card profile filenames
#define BAND_18_21_FILE             "/profiles/band_18_21.json"
#define BAND_21_24_FILE             "/profiles/band_21_24.json"
#define BAND_24_27_FILE             "/profiles/band_24_27.json"
#define BAND_27_30_FILE             "/profiles/band_27_30.json"

#define PROFILE_JSON_MAX_SIZE       1024      // Max JSON file size

// ============================================
// ADAPTIVE LEARNING (GH-AL-005, GH-AL-006)
// ============================================

#define DEFAULT_EMA_WEIGHT          0.30f     // 30% new data, 70% historical
#define EMA_WEIGHT_MIN              0.10f
#define EMA_WEIGHT_MAX              0.50f
#define CONFIDENCE_MAX              0.90f     // Cap confidence at 90%

// ============================================
// ACTIVE CALIBRATION (v1.3)
// ============================================

// Phase durations
#define CALIBRATION_STABILIZE_SEC       300   // 5 min settle time
#define CALIBRATION_RISE_MAX_SEC        600   // 10 min maximum rise time
#define CALIBRATION_TOTAL_SEC           1200  // 20 min total calibration window

// Plateau detection (slope-based, two-window confirmation)
#define CALIBRATION_PLATEAU_SLOPE_PCT   0.5f  // % RH per minute — below this = plateau
#define CALIBRATION_PLATEAU_WINDOW_SEC  120   // First slope evaluation window (2 min)
#define CALIBRATION_PLATEAU_CONFIRM_SEC 120   // Second window for confirmation (2 min)
// Total plateau confirmation time: WINDOW + CONFIRM = up to 240s (4 min)

// Minimum expected RH rise during active calibration.
// If the humidifier + air assist cannot raise RH by at least this much,
// the calibration run is discarded (indicates empty reservoir, dead pump, etc.)
#define CALIBRATION_MIN_EXPECTED_RISE_PCT  3.0f

// Minimum total RH swing during calibration to consider the run valid.
#define CALIBRATION_MIN_RH_DELTA        2.0f

// ============================================
// PID CONTROLLER (v1.3)
// ============================================

// Auto-enable PID when confidence reaches this threshold
#define PID_AUTO_ENABLE_CONFIDENCE      0.70f

// Duty cycle window and relay protection
#define PID_CYCLE_MS                    30000 // 30 second duty cycle window
#define PID_MIN_RELAY_ON_MS             3000  // Minimum humidifier ON time (prevents short-cycling)
#define PID_MIN_RELAY_OFF_MS            3000  // Minimum OFF time (prevents relay chatter)
#define PID_DUTY_CLAMP_LOW_PCT          5     // Below this = force OFF
#define PID_DUTY_CLAMP_HIGH_PCT         95    // Above this = force full ON
#define PID_BOOST_THRESHOLD_PCT         5.0f  // Outside ±5% of setpoint = run 100% ON (boost mode)

// PID sample time (matches sensor poll interval for consistent dt)
#define PID_SAMPLE_TIME_MS              2000

// Default PID gains (used when no calibration profile exists)
#define PID_DEFAULT_KP                  0.5f
#define PID_DEFAULT_KI                  0.01f
#define PID_DEFAULT_KD                  0.1f

// ============================================
// RELAY CONTROL
// ============================================

#define RELAY_HOH                       0     // Humidifier relay
#define RELAY_AIR_ASSIST                1     // Air assist fan relay
#define RELAY_HEATER                    2     // Heater relay
#define RELAY_LIGHTS                    3     // Lights relay

#define RELAY_COUNT                     4

// ============================================
// SAFETY GUARDRAILS
// ============================================

#define DEFAULT_HUM_FLOOR               40.0f
#define DEFAULT_HUM_CEILING             80.0f
#define DEFAULT_TEMP_FLOOR              15.0f
#define DEFAULT_TEMP_CEILING            35.0f

#define SAFETY_HUM_CEILING_ABSOLUTE     90.0f  // Hard stop regardless of config
#define SAFETY_TEMP_CEILING_ABSOLUTE    40.0f  // Hard stop regardless of config

// ============================================
// DEFAULT RECOVERY RATE
// ============================================

// Conservative estimate used when no calibration profile exists
// 1% RH per minute = 0.0167% per second
#define DEFAULT_RECOVERY_RATE_PCT_PER_SEC  0.0167f

// ============================================
// SD CARD
// ============================================

#define SD_CS_PIN                       5
#define SD_CARD_DETECT_PIN              -1    // Not used

// ============================================
// WEB UI / API
// ============================================

#define WEB_PORT                        80
#define API_PATH                        "/api"
#define WS_PATH                         "/ws"

// ============================================
// NETWORK
// ============================================

#define WIFI_RETRY_INTERVAL_MS          30000
#define WIFI_MAX_RETRIES                5

#endif // CONFIG_H