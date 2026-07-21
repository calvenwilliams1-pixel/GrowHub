/*
   automation.cpp
   GrowHub32 - Main Automation Control Loop
   Version: 1.0.7
   Revision: ALL issues resolved:
             - Watchdog cooldown recovery now works while tripped (no circular dependency)
             - Cooldown check runs continuously when OFF, not just on transitions
             - Once tripped, system auto-recovers after 5 minutes of OFF time
             - No manual reboot required
             - All other fixes retained

   Control Flow:
   1. Read SCD40 sensors (temp, humidity, CO2)
   2. Update system state (fault detection, last known values)
   3. If calibration active:
        Run adaptive_updateCalibration() (PID is disabled internally)
   4. Else:
        Auto-tune PID if band changes or profile updates
        Execute humidity control (single unified function)
   5. Update Web UI via WebSocket push
   6. Log data to SD card
   7. Handle WiFi/OTA tasks
*/

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "sensors.h"
#include "relay_manager.h"
#include "pid_controller.h"
#include "adaptive.h"
#include "sd_logger.h"
#include "rtc_handler.h"
#include "network.h"
#include "web_ui.h"

// ============================================
// External Declarations
// ============================================

extern SystemState g_systemState;
extern AutomationThresholds g_thresholds;
extern void automation_profileUpdated();

// ============================================
// Global PID Instance
// ============================================

PIDController g_humidityPID;

// ============================================
// Relay Protection State
// ============================================

static struct {
    unsigned long lastOnTime[RELAY_COUNT];
    unsigned long lastOffTime[RELAY_COUNT];
    bool relayState[RELAY_COUNT];
    bool initialized[RELAY_COUNT];
    unsigned long dutyCycleStartTime;
} g_relayProtection;

// ============================================
// Watchdog State
// ============================================

static struct {
    unsigned long accumulatedOnTime;   // Total ON time from completed segments (ms)
    unsigned long windowStartTime;     // Start of accumulation window
    unsigned long lastOnTransitionTime;// When current ON segment started
    unsigned long offDurationStart;    // When OFF period started (for cooldown tracking)
    bool watchdogTripped;              // Flag for watchdog state
    bool prevRelayState;               // Previous relay state for transition detection
} g_watchdog;

#define WATCHDOG_WINDOW_MS    7200000UL   // 2 hours
#define WATCHDOG_MAX_ON_MS    1800000UL   // 30 minutes max ON within window
#define WATCHDOG_COOLDOWN_MS  300000UL    // 5 minutes OFF to reset

// ============================================
// Timing
// ============================================

static unsigned long g_lastSensorRead = 0;
static unsigned long g_lastLogWrite = 0;
static unsigned long g_lastWebUpdate = 0;
static unsigned long g_lastWiFiCheck = 0;
static unsigned long g_lastCalibrationRun = 0;
static unsigned long g_lastPidCompute = 0;

// ============================================
// Profile Version Tracking
// ============================================

static uint32_t g_profileVersion = 1;
static uint32_t g_lastProfileVersion = 0;

// ============================================
// Persistent State Flags
// ============================================

static bool g_sensorFaultAlerted = false;
static bool g_ceilingAlerted = false;
static bool g_boostActive = false;

// ============================================
// Private Helpers
// ============================================

static void applyRelayProtection(int relayIndex, bool enable) {
    unsigned long now = millis();
    
    // Initialize timestamps on first use
    if (!g_relayProtection.initialized[relayIndex]) {
        g_relayProtection.lastOnTime[relayIndex] = now - PID_MIN_RELAY_ON_MS;
        g_relayProtection.lastOffTime[relayIndex] = now - PID_MIN_RELAY_OFF_MS;
        g_relayProtection.initialized[relayIndex] = true;
        // Continue to apply the requested state
    }
    
    if (enable) {
        if (now - g_relayProtection.lastOffTime[relayIndex] < PID_MIN_RELAY_OFF_MS) {
            return;
        }
        if (g_relayProtection.relayState[relayIndex]) {
            return;
        }
        relayManager_setRelay(relayIndex, true);
        g_relayProtection.relayState[relayIndex] = true;
        g_relayProtection.lastOnTime[relayIndex] = now;
    } else {
        if (now - g_relayProtection.lastOnTime[relayIndex] < PID_MIN_RELAY_ON_MS) {
            return;
        }
        if (!g_relayProtection.relayState[relayIndex]) {
            return;
        }
        relayManager_setRelay(relayIndex, false);
        g_relayProtection.relayState[relayIndex] = false;
        g_relayProtection.lastOffTime[relayIndex] = now;
    }
}

static float computeDutyCycle(float output) {
    if (output < PID_DUTY_CLAMP_LOW_PCT) return 0.0f;
    if (output > PID_DUTY_CLAMP_HIGH_PCT) return 100.0f;
    return output;
}

static void updateRelaysFromPID(float dutyCycle) {
    unsigned long now = millis();
    
    if (g_relayProtection.dutyCycleStartTime == 0 || 
        (now - g_relayProtection.dutyCycleStartTime >= PID_CYCLE_MS)) {
        g_relayProtection.dutyCycleStartTime = now;
    }
    
    unsigned long windowElapsed = now - g_relayProtection.dutyCycleStartTime;
    unsigned long onTimeTarget = (unsigned long)((dutyCycle / 100.0f) * PID_CYCLE_MS);
    
    bool shouldBeOn = (dutyCycle > 0.0f) && (windowElapsed < onTimeTarget);
    
    applyRelayProtection(RELAY_HOH, shouldBeOn);
    applyRelayProtection(RELAY_AIR_ASSIST, shouldBeOn);
}

static void pid_setTunings(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// ============================================
// Watchdog Core (with self-recovery cooldown)
// ============================================

static void updateWatchdog() {
    unsigned long now = millis();
    bool currentRelayState = g_relayProtection.relayState[RELAY_HOH];
    bool transitionDetected = (currentRelayState != g_watchdog.prevRelayState);
    
    // --- If watchdog is tripped, check for cooldown recovery ---
    // This runs continuously while OFF, not just on transitions.
    // Fixes the circular dependency: recovery doesn't require an ON transition.
    if (g_watchdog.watchdogTripped && !currentRelayState) {
        if (g_watchdog.offDurationStart == 0) {
            g_watchdog.offDurationStart = now;
        } else if (now - g_watchdog.offDurationStart >= WATCHDOG_COOLDOWN_MS) {
            // Cooldown complete — reset everything and clear trip
            g_watchdog.windowStartTime = 0;
            g_watchdog.accumulatedOnTime = 0;
            g_watchdog.watchdogTripped = false;
            g_watchdog.offDurationStart = 0;
            Serial.println(F("[WATCHDOG] Cooldown complete — trip cleared, PID re-enabled"));
        }
        // Save state and return early (PID is blocked until cooldown completes)
        g_watchdog.prevRelayState = currentRelayState;
        return;
    }
    
    // --- ON → OFF Transition: commit completed segment ---
    if (transitionDetected && !currentRelayState) {
        if (g_watchdog.lastOnTransitionTime > 0) {
            g_watchdog.accumulatedOnTime += (now - g_watchdog.lastOnTransitionTime);
        }
        g_watchdog.offDurationStart = now;
    }
    
    // --- OFF → ON Transition: start new segment, check cooldown ---
    if (transitionDetected && currentRelayState) {
        g_watchdog.lastOnTransitionTime = now;
        g_watchdog.offDurationStart = 0;  // Clear OFF timer
        
        if (g_watchdog.windowStartTime == 0) {
            g_watchdog.windowStartTime = now;
        }
    }
    
    // --- Ongoing ON accumulation ---
    if (currentRelayState) {
        // Check 2-hour window expiration
        if (g_watchdog.windowStartTime > 0 && 
            (now - g_watchdog.windowStartTime >= WATCHDOG_WINDOW_MS)) {
            g_watchdog.windowStartTime = now;
            g_watchdog.accumulatedOnTime = 0;
            g_watchdog.lastOnTransitionTime = now;
        }
        
        // Calculate total ON time = completed segments + current active segment
        unsigned long currentSegmentOnTime = now - g_watchdog.lastOnTransitionTime;
        unsigned long totalOnTime = g_watchdog.accumulatedOnTime + currentSegmentOnTime;
        
        // Check safety limit
        if (totalOnTime >= WATCHDOG_MAX_ON_MS && !g_watchdog.watchdogTripped) {
            Serial.println(F("[WATCHDOG] Humidifier ON time exceeded 30 min in 2h window"));
            String alert = String("Humidifier ran ") + 
                          String(totalOnTime / 60000) + 
                          String(" min in 2h — forcing OFF");
            network_sendAlert("Humidifier Watchdog", alert.c_str());
            g_watchdog.watchdogTripped = true;
            g_watchdog.offDurationStart = now;  // Start cooldown timer immediately
            
            // Force relays OFF
            applyRelayProtection(RELAY_HOH, false);
            applyRelayProtection(RELAY_AIR_ASSIST, false);
            g_systemState.currentDutyCycle = 0.0f;
            g_systemState.pidOutput = 0.0f;
        }
    } else {
        // Relay is OFF — update cooldown timer (will be checked on next loop)
        if (!g_watchdog.watchdogTripped && g_watchdog.offDurationStart == 0) {
            // First OFF after an ON period — set start time
            g_watchdog.offDurationStart = now;
        }
    }
    
    // Save state for next iteration
    g_watchdog.prevRelayState = currentRelayState;
}

// ============================================
// Humidity Control Core (single unified function)
// ============================================

static void executeHumidityControl() {
    unsigned long now = millis();
    
    if (!pid_isEnabled(&g_humidityPID) || !sensors_isHumidityValid()) {
        applyRelayProtection(RELAY_HOH, false);
        applyRelayProtection(RELAY_AIR_ASSIST, false);
        g_systemState.currentDutyCycle = 0.0f;
        g_systemState.pidOutput = 0.0f;
        
        if (!sensors_isHumidityValid() && !g_sensorFaultAlerted) {
            network_sendAlert("Sensor Fault", "Humidity sensor is not responding");
            g_sensorFaultAlerted = true;
        }
        return;
    }
    
    float setpoint = pid_getSetpoint(&g_humidityPID);
    float currentRH = g_systemState.currentHumidity;
    
    // --- Hard safety cutoff: humidity ceiling ---
    if (currentRH >= g_thresholds.humCeiling) {
        applyRelayProtection(RELAY_HOH, false);
        applyRelayProtection(RELAY_AIR_ASSIST, false);
        g_systemState.currentDutyCycle = 0.0f;
        g_systemState.pidOutput = 0.0f;
        
        if (!g_ceilingAlerted) {
            String alert = String("Humidity at ") + String(currentRH, 1) + 
                          String("% — humidifier forced OFF");
            network_sendAlert("Humidity Ceiling Reached", alert.c_str());
            g_ceilingAlerted = true;
        }
        return;
    }
    
    // Ceiling cleared — reset alert flag
    if (g_ceilingAlerted) {
        g_ceilingAlerted = false;
    }
    
    // --- Watchdog: track accumulated ON time ---
    updateWatchdog();
    
    // If watchdog tripped, skip PID (relay is forced OFF, cooldown running)
    if (g_watchdog.watchdogTripped) {
        return;
    }
    
    // --- Compute PID only at sample interval ---
    if (now - g_lastPidCompute < PID_SAMPLE_TIME_MS) {
        return;
    }
    g_lastPidCompute = now;
    
    // --- Normal PID computation ---
    float dutyCycle = pid_compute(&g_humidityPID, currentRH);
    float finalDuty = computeDutyCycle(dutyCycle);
    
    // --- Boost mode (only when RH is BELOW setpoint) ---
    float deficit = setpoint - currentRH;
    if (deficit > PID_BOOST_THRESHOLD_PCT && finalDuty < 100.0f) {
        finalDuty = 100.0f;
        if (!g_boostActive) {
            Serial.println(F("[BOOST] Full ON — RH below setpoint by >5%"));
            g_boostActive = true;
        }
    } else {
        if (g_boostActive) {
            g_boostActive = false;
            Serial.println(F("[BOOST] Exited boost mode"));
        }
    }
    
    // Update relays
    updateRelaysFromPID(finalDuty);
    
    // Store state for Web UI
    g_systemState.currentDutyCycle = finalDuty;
    g_systemState.pidOutput = dutyCycle;
}

// ============================================
// PID Auto-Tuning
// ============================================

static void autoTunePID() {
    BandProfile* profile = adaptive_getActiveProfile();
    
    if (!profile || !profile->valid) {
        pid_setTunings(&g_humidityPID, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
        Serial.println(F("[AUTO] Using default gains (no valid profile)"));
        return;
    }
    
    if (profile->confidenceScore >= PID_AUTO_ENABLE_CONFIDENCE) {
        float oldKp = g_humidityPID.kp;
        float oldKi = g_humidityPID.ki;
        float oldKd = g_humidityPID.kd;
        
        pid_tuneFromProfile(&g_humidityPID,
                           profile->avgRecoveryRate,
                           profile->riseTimeSeconds,
                           profile->confidenceScore
        );
        
        Serial.print(F("[AUTO] PID tuned: Kp="));
        Serial.print(g_humidityPID.kp, 2);
        Serial.print(F(" (was "));
        Serial.print(oldKp, 2);
        Serial.print(F(") Ki="));
        Serial.print(g_humidityPID.ki, 3);
        Serial.print(F(" (was "));
        Serial.print(oldKi, 3);
        Serial.print(F(") Kd="));
        Serial.print(g_humidityPID.kd, 2);
        Serial.print(F(" (was "));
        Serial.print(oldKd, 2);
        Serial.println(F(")"));
    } else {
        pid_setTunings(&g_humidityPID, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
        Serial.print(F("[AUTO] Using default gains (confidence="));
        Serial.print(profile->confidenceScore, 2);
        Serial.println(F(" < threshold)"));
    }
}

// ============================================
// Main Setup
// ============================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n========================================"));
    Serial.print(F("GrowHub32 v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" Starting..."));
    Serial.println(F("========================================"));
    
    // 1. Initialize SD card
    if (!sdLogger_init()) {
        Serial.println(F("[WARN] SD card initialization failed or unavailable!"));
    }
    
    // 2. Initialize RTC
    rtc_init();
    
    // 3. Initialize sensors (SCD40)
    sensors_init();
    
    // 4. Initialize relay manager (handles brownout-safe defaults internally)
    relayManager_init();
    
    // 5. Initialize adaptive learning
    adaptive_init();
    adaptive_loadProfiles();
    g_profileVersion = 1;
    
    // 6. Initialize PID controller with defaults
    pid_init(&g_humidityPID,
             PID_DEFAULT_KP,
             PID_DEFAULT_KI,
             PID_DEFAULT_KD,
             PID_SAMPLE_TIME_MS,
             0.0f,
             100.0f
    );
    
    // 7. Set initial setpoint
    float initialSetpoint = (g_thresholds.humHoHFloor + g_thresholds.humCeiling) / 2.0f;
    pid_setSetpoint(&g_humidityPID, initialSetpoint);
    g_systemState.setpointHumidity = initialSetpoint;
    
    // 8. Initialize network (WiFi)
    network_init();
    
    // 9. Initialize Web UI
    webUI_init();
    
    // 10. Initialize relay protection state
    for (int i = 0; i < RELAY_COUNT; i++) {
        g_relayProtection.lastOnTime[i] = 0;
        g_relayProtection.lastOffTime[i] = 0;
        g_relayProtection.relayState[i] = false;
        g_relayProtection.initialized[i] = false;
    }
    g_relayProtection.dutyCycleStartTime = 0;
    
    // 11. Initialize watchdog state
    g_watchdog.accumulatedOnTime = 0;
    g_watchdog.windowStartTime = 0;
    g_watchdog.lastOnTransitionTime = 0;
    g_watchdog.offDurationStart = 0;
    g_watchdog.watchdogTripped = false;
    g_watchdog.prevRelayState = false;
    
    // 12. Reset persistent state flags
    g_sensorFaultAlerted = false;
    g_ceilingAlerted = false;
    g_boostActive = false;
    g_lastPidCompute = 0;
    
    Serial.println(F("========================================"));
    Serial.println(F("GrowHub32 Initialization Complete"));
    Serial.println(F("========================================"));
}

// ============================================
// Main Loop
// ============================================

void loop() {
    unsigned long now = millis();
    
    // ============================================
    // 1. Read Sensors (every 2 seconds)
    // ============================================
    if (now - g_lastSensorRead >= SCD40_MEASURE_INTERVAL_MS) {
        g_lastSensorRead = now;
        sensors_update();
        
        bool tempValid = sensors_isTemperatureValid();
        bool humValid = sensors_isHumidityValid();
        
        if (tempValid && humValid) {
            g_systemState.currentTemp = sensors_getTemperature();
            g_systemState.currentHumidity = sensors_getHumidity();
            g_systemState.lastKnownGoodTemp = g_systemState.currentTemp;
            g_systemState.lastKnownGoodHumidity = g_systemState.currentHumidity;
            g_systemState.tempSensorFault = false;
            g_systemState.humiditySensorFault = false;
            
            if (g_sensorFaultAlerted) {
                g_sensorFaultAlerted = false;
                Serial.println(F("[SENSOR] Sensors recovered"));
            }
        } else {
            g_systemState.tempSensorFault = !tempValid;
            g_systemState.humiditySensorFault = !humValid;
        }
        
        // Update temperature band
        g_systemState.currentBand = adaptive_getCurrentBand();
    }
    
    // ============================================
    // 2. Handle Calibration
    // ============================================
    if (adaptive_isCalibrating()) {
        if (now - g_lastCalibrationRun >= 100) {
            g_lastCalibrationRun = now;
            adaptive_updateCalibration();
        }
    } else {
        // ============================================
        // 3. Auto-Tune PID
        // ============================================
        static uint8_t lastBand = 0xFF;
        uint8_t currentBand = g_systemState.currentBand;
        
        if (currentBand != lastBand || g_profileVersion != g_lastProfileVersion) {
            lastBand = currentBand;
            g_lastProfileVersion = g_profileVersion;
            
            autoTunePID();
            
            float newSetpoint = (g_thresholds.humHoHFloor + g_thresholds.humCeiling) / 2.0f;
            pid_setSetpoint(&g_humidityPID, newSetpoint);
            g_systemState.setpointHumidity = newSetpoint;
        }
        
        // ============================================
        // 4. Execute Humidity Control
        // ============================================
        executeHumidityControl();
    }
    
    // ============================================
    // 5. Update Web UI (every 1 second)
    // ============================================
    if (now - g_lastWebUpdate >= 1000) {
        g_lastWebUpdate = now;
        webUI_pushUpdates();
    }
    
    // ============================================
    // 6. Log to SD Card (every 60 seconds)
    // ============================================
    if (now - g_lastLogWrite >= 60000) {
        g_lastLogWrite = now;
        if (sdLogger_isAvailable()) {
            sdLogger_writeData();
        }
    }
    
    // ============================================
    // 7. Handle WiFi / OTA (every 5 seconds)
    // ============================================
    if (now - g_lastWiFiCheck >= 5000) {
        g_lastWiFiCheck = now;
        network_update();
    }
    
    // ============================================
    // 8. Handle Web Server Client Requests
    // ============================================
    webUI_handleClient();
    
    delay(10);
}

// ============================================
// External API: Notify profile version change
// ============================================

void automation_profileUpdated() {
    g_profileVersion++;
    Serial.print(F("[AUTO] Profile version updated to "));
    Serial.println(g_profileVersion);
}