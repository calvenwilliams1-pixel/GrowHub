/**
 * @file automation.cpp
 * @brief GrowHub32 v1.3 — Unified Automation Controller
 * 
 * This file is the single source of truth for all environmental control
 * decisions. It replaces the two divergent automation.cpp files from
 * earlier versions (one PID-based, one threshold-based).
 * 
 * ARCHITECTURE:
 *   The system has two control strategies for humidity:
 * 
 *   1. PID MODE (high confidence) — When the current temperature band's
 *      calibration profile has confidence >= PID_AUTO_ENABLE_CONFIDENCE,
 *      a PID controller modulates HOH duty cycle with Air Assist bursts
 *      during the ON portion. Boost mode engages when RH is >5% below
 *      setpoint (full ON until recovered).
 * 
 *   2. BANG-BANG MODE (low confidence / fallback) — When confidence is
 *      below threshold or no profile exists, simple threshold control:
 *      HOH ON below HOH floor, Air Assist joins below Assist floor,
 *      everything OFF at ceiling.
 * 
 *   CO2 control, night mode enforcement, temperature alerts, and manual
 *   overrides run identically regardless of which humidity mode is active.
 * 
 * CONTROL FLOW (every loop iteration):
 *   1. Night mode check (lock out compressor + air assist if active)
 *   2. Temperature alerts
 *   3. CO2 control (exhaust fan)
 *   4. Humidity control:
 *      a) Update active temperature band
 *      b) Ceiling check -> everything OFF
 *      c) Below Assist floor -> emergency recovery (HOH ON, Air Assist bursts)
 *      d) Below HOH floor -> HOH ON only
 *      e) Maintenance zone -> PID if confident, else bang-bang OFF
 *   5. Humidifier runtime watchdog (limits HOH ON time to 30min/2hr)
 * 
 * OWNERSHIP:
 *   - PID controller state: this file (g_humidityPID)
 *   - Humidifier watchdog: this file (g_watchdog)
 *   - Relay actuation: delegated to relay_manager via applyRelayProtection()
 *   - Thresholds: this file (g_thresholds), mutex-protected for Web UI access
 *   - Override state: this file, mutex-protected, auto-expiry via getter functions
 *   - Calibration: adaptive.cpp
 *   - Sensor data: sensors.cpp via g_systemState
 *   - Active band tracking: this file (g_systemState.currentBand)
 * 
 * v1.3 fixes applied (duck review rounds 1-8):
 *   - All g_systemState writes wrapped in portENTER_CRITICAL
 *   - g_thresholds and override state mutex-protected
 *   - Override auto-expiry preserved (getter functions handle timeout + clear)
 *   - currentBand updated every loop iteration, rate-limited to 5s
 *   - All String heap allocations replaced with snprintf stack buffers
 *   - Night mode guard in runAirAssistBursts() with burst state reset
 *   - Watchdog check in emergency recovery path
 *   - suspendPID() clears telemetry and is called from all exit paths
 *   - Transition-based emergency recovery (g_inEmergencyRecovery flag)
 *   - autoTunePID() rate-limited, called every loop when PID active
 *   - PID suspended on sensor fault and ceiling in executeHumidityControl()
 *   - applyRelayProtection() force parameter for terminal safety shutdowns
 *   - stopHumidityOutputs() helper eliminates duplicate shutdown code
 *   - Temperature alert first-fire fixed (was suppressed first 5 min)
 *   - g_lastPidCompute reset on watchdog clear + override deactivation
 *   - lastKnownGoodHumidity freshness check (safe mode after 30s fault)
 *   - Sensor read atomicity (validity + value under single critical section)
 *   - Watchdog millis() wrap detection
 *   - First-boot relay delay fixed (lastChange==0 sentinel)
 *   - pid_setTunings renamed to automation_setPidTunings (no shadowing)
 *   - Fixed swapped MIN_RELAY_ON/MIN_RELAY_OFF constants
 *   - dutyCycleStartTime and airAssistBurstStart wrap detection
 *   - sensorFaultTimer + lastKnownGoodHumidity under single critical section
 *   - network_sendAlert() non-blocking/heap-free assumption documented
 *   - Assist timing validation uses bounds + ON<=OFF rule
 *   - CO2 hysteresis: 100ppm min gap between high/low thresholds
 *   - Null check on automation_updateThresholds()
 *   - PID telemetry updated during emergency/bang-bang mode (shows 100%)
 */

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "system_state.h"
#include "sensors.h"
#include "relay_manager.h"
#include "pid_controller.h"
#include "adaptive.h"
#include "automation.h"
#include "rtc_handler.h"
#include "network.h"

// ============================================================
// External Declarations
// ============================================================

// Assumes network_sendAlert() is non-blocking and heap-free.
// Called from real-time control loop — must not allocate or block.
extern void network_sendAlert(const char* title, const char* message);
extern portMUX_TYPE g_stateMux;

// ============================================================
// Forward Declarations
// ============================================================

static void runAirAssistBursts();
static void suspendPID();
static void stopHumidityOutputs();
static void applyRelayProtection(uint8_t relayIndex, bool desiredState, bool force = false);
static void updateWatchdog();
static void autoTunePID();
static void executeHumidityControl();

// ============================================================
// Global PID Instance
// ============================================================

 PIDController g_humidityPID;

// ============================================================
// Relay Protection State (min ON/OFF timing for HOH + Air Assist)
// ============================================================

static struct {
    unsigned long hohLastChange;
    unsigned long assistLastChange;
    unsigned long dutyCycleStartTime;
} g_relayTiming;

// ============================================================
// Humidifier Runtime Watchdog
// ============================================================
// Limits total HOH ON time to WATCHDOG_HOH_MAX_ON_MS per
// WATCHDOG_HOH_WINDOW_MS. This is separate from the ESP32
// hardware task watchdog (esp_task_wdt in safety.cpp).
// ============================================================

static struct {
    unsigned long accumulatedOnTime;
    unsigned long windowStartTime;
    unsigned long lastOnTransitionTime;
    unsigned long offDurationStart;
    bool watchdogTripped;
    bool prevRelayState;
} g_watchdog;

// ============================================================
// Automation Thresholds
// ============================================================

static AutomationThresholds g_thresholds;

// ============================================================
// Air Assist Burst State
// ============================================================

static bool g_airAssistBurstActive = false;
static unsigned long g_airAssistBurstStart = 0;
static bool g_airAssistInOnPhase = false;

// ============================================================
// Emergency Recovery State (transition-based PID reset)
// ============================================================

static bool g_inEmergencyRecovery = false;

// ============================================================
// Manual Override State
// ============================================================

static bool g_humidityOverrideActive = false;
static unsigned long g_humidityOverrideStart = 0;
static bool g_co2OverrideActive = false;
static unsigned long g_co2OverrideStart = 0;

// ============================================================
// PID Runtime State
// ============================================================

static unsigned long g_lastPidCompute = 0;
static bool g_boostActive = false;
static float g_lastTunedConfidence = 0.0f;

// ============================================================
// Private Helpers
// ============================================================

/**
 * @brief Suspend PID controller, reset internal state, and clear telemetry.
 * 
 * Called whenever the system exits PID mode — emergency recovery,
 * ceiling shutdown, watchdog trip, calibration start, manual override,
 * sensor fault. Ensures the PID wakes up clean with no stale integral,
 * derivative, or dashboard telemetry.
 */
static void suspendPID() {
    pid_setEnabled(&g_humidityPID, false);
    pid_reset(&g_humidityPID);
    g_lastPidCompute = 0;
    g_boostActive = false;

    // Clear PID telemetry so dashboard shows inactive state
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.currentDutyCycle = 0.0f;
    g_systemState.pidOutput = 0.0f;
    portEXIT_CRITICAL(&g_stateMux);
}

/**
 * @brief Force all humidity outputs OFF immediately.
 * 
 * Bypasses minimum ON/OFF timing — used for terminal safety shutdowns
 * (ceiling, sensor fault, watchdog trip, override activation).
 * Also clears burst state so morning/mode transitions start clean.
 */
static void stopHumidityOutputs() {
    applyRelayProtection(RELAY_HOH, false, true);
    applyRelayProtection(RELAY_AIR_ASSIST, false, true);
    g_airAssistBurstActive = false;
    g_airAssistInOnPhase = false;
}

/**
 * @brief Apply relay state with minimum ON/OFF timing protection.
 * 
 * This is a thin wrapper around relayManager_setRelay() that adds
 * minimum pulse width enforcement for HOH and Air Assist relays.
 * Hardware ownership remains in relay_manager — all GPIO writes
 * occur through relayManager_setRelay(). This function only adds
 * timing policy; it never manipulates GPIO directly.
 * 
 * @param relayIndex  RELAY_HOH or RELAY_AIR_ASSIST
 * @param desiredState true = ON, false = OFF
 * @param force       If true, bypass min timing (terminal shutdowns only)
 */
static void applyRelayProtection(uint8_t relayIndex, bool desiredState, bool force) {
    if (relayIndex != RELAY_HOH && relayIndex != RELAY_AIR_ASSIST) {
        relayManager_setRelay(relayIndex, desiredState);
        return;
    }

    bool currentlyOn = relayManager_isRelayOn(relayIndex);
    if (desiredState == currentlyOn) return;

    unsigned long now = millis();
    unsigned long lastChange = (relayIndex == RELAY_HOH) ?
        g_relayTiming.hohLastChange : g_relayTiming.assistLastChange;

    // When turning ON: relay is currently OFF, must have been OFF long enough → OFF_MS
    // When turning OFF: relay is currently ON, must have been ON long enough → ON_MS
    unsigned long minTime = desiredState ? PID_MIN_RELAY_OFF_MS : PID_MIN_RELAY_ON_MS;

    // lastChange==0 means uninitialized — allow immediate first change
    // force=true bypasses timing for terminal safety shutdowns
    if (!force && (lastChange != 0) && (now - lastChange < minTime)) {
        return;
    }

    relayManager_setRelay(relayIndex, desiredState);

    if (relayIndex == RELAY_HOH) {
        g_relayTiming.hohLastChange = now;
    } else {
        g_relayTiming.assistLastChange = now;
    }
}

/**
 * @brief Convert PID output (0-100%) to a duty cycle with clamping.
 */
static float computeDutyCycle(float output) {
    if (output < PID_DUTY_CLAMP_LOW_PCT) return 0.0f;
    if (output > PID_DUTY_CLAMP_HIGH_PCT) return 100.0f;
    return output;
}

/**
 * @brief Apply PID duty cycle to relays within the 30-second window.
 */
static void updateRelaysFromPID(float dutyCycle) {
    unsigned long now = millis();

    // Reset cycle if uninitialized, expired, or millis() wrapped
    if (g_relayTiming.dutyCycleStartTime == 0 ||
        now < g_relayTiming.dutyCycleStartTime ||
        (now - g_relayTiming.dutyCycleStartTime >= PID_CYCLE_MS)) {
        g_relayTiming.dutyCycleStartTime = now;
    }

    unsigned long windowElapsed = now - g_relayTiming.dutyCycleStartTime;
    unsigned long onTimeTarget = (unsigned long)((dutyCycle / 100.0f) * PID_CYCLE_MS);
    bool shouldBeOn = (dutyCycle > 0.0f) && (windowElapsed < onTimeTarget);

    applyRelayProtection(RELAY_HOH, shouldBeOn);

    if (shouldBeOn) {
        runAirAssistBursts();
    } else {
        if (g_airAssistBurstActive || relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
            applyRelayProtection(RELAY_AIR_ASSIST, false);
            g_airAssistBurstActive = false;
            g_airAssistInOnPhase = false;
        }
    }
}

/**
 * @brief Set PID gains from a helper (used during auto-tuning).
 */
static void automation_setPidTunings(PIDController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// ============================================================
// Humidifier Runtime Watchdog
// ============================================================

static void updateWatchdog() {
    unsigned long now = millis();

    // Handle millis() wraparound (~49.7 days) — reset timing references
    if (now < g_watchdog.windowStartTime) {
        g_watchdog.windowStartTime = now;
        g_watchdog.accumulatedOnTime = 0;
    }
    if (now < g_watchdog.lastOnTransitionTime) {
        g_watchdog.lastOnTransitionTime = now;
    }
    if (now < g_watchdog.offDurationStart) {
        g_watchdog.offDurationStart = now;
    }

    bool currentRelayState = relayManager_isRelayOn(RELAY_HOH);
    bool transitionDetected = (currentRelayState != g_watchdog.prevRelayState);

    // If watchdog is tripped and relay is OFF, check for cooldown recovery
    if (g_watchdog.watchdogTripped && !currentRelayState) {
        if (g_watchdog.offDurationStart == 0) {
            g_watchdog.offDurationStart = now;
        } else if (now - g_watchdog.offDurationStart >= WATCHDOG_COOLDOWN_MS) {
            g_watchdog.windowStartTime = 0;
            g_watchdog.accumulatedOnTime = 0;
            g_watchdog.watchdogTripped = false;
            g_watchdog.offDurationStart = 0;
            g_lastPidCompute = 0;  // Force immediate PID computation on recovery
            Serial.println(F("[WATCHDOG] Cooldown complete — trip cleared"));
        }
        g_watchdog.prevRelayState = currentRelayState;
        return;
    }

    // ON -> OFF transition: commit completed segment
    if (transitionDetected && !currentRelayState) {
        if (g_watchdog.lastOnTransitionTime > 0) {
            g_watchdog.accumulatedOnTime += (now - g_watchdog.lastOnTransitionTime);
        }
        g_watchdog.offDurationStart = now;
    }

    // OFF -> ON transition: start new segment
    if (transitionDetected && currentRelayState) {
        g_watchdog.lastOnTransitionTime = now;
        g_watchdog.offDurationStart = 0;
        if (g_watchdog.windowStartTime == 0) {
            g_watchdog.windowStartTime = now;
        }
    }

    // Ongoing ON: check limits
    if (currentRelayState) {
        if (g_watchdog.windowStartTime > 0 &&
            (now - g_watchdog.windowStartTime >= WATCHDOG_HOH_WINDOW_MS)) {
            g_watchdog.windowStartTime = now;
            g_watchdog.accumulatedOnTime = 0;
            g_watchdog.lastOnTransitionTime = now;
        }

        unsigned long currentSegmentOnTime = now - g_watchdog.lastOnTransitionTime;
        unsigned long totalOnTime = g_watchdog.accumulatedOnTime + currentSegmentOnTime;

        if (totalOnTime >= WATCHDOG_HOH_MAX_ON_MS && !g_watchdog.watchdogTripped) {
            Serial.println(F("[WATCHDOG] HOH ON time exceeded 30 min in 2h window"));

            char alert[64];
            snprintf(alert, sizeof(alert),
                     "Humidifier ran %lu min in 2h — forcing OFF",
                     totalOnTime / 60000UL);
            network_sendAlert("Humidifier Watchdog", alert);

            g_watchdog.watchdogTripped = true;
            g_watchdog.offDurationStart = now;

            // Suspend PID so it doesn't resume with stale integral after cooldown
            if (pid_isEnabled(&g_humidityPID)) {
                suspendPID();
            }

            stopHumidityOutputs();
        }
    }

    g_watchdog.prevRelayState = currentRelayState;
}

// ============================================================
// Air Assist Burst Control
// ============================================================

/**
 * @brief Run Air Assist in configurable burst cycles.
 * 
 * Air Assist is NEVER continuous. It alternates between
 * assistOnSec ON and assistOffSec OFF. This prevents
 * oversaturation and crop damage.
 * 
 * Night mode guard: if night mode is active, Air Assist is
 * locked out by noise ordinance. Burst state is reset to
 * prevent stale timers on morning transition.
 */
static void runAirAssistBursts() {
    // Night mode lockout — Air Assist is loud, compressor is loud.
    // HOH is silent and continues to operate normally at night.
    if (g_systemState.nightModeActive) {
        // Reset burst state so morning starts clean
        if (g_airAssistBurstActive || relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
            applyRelayProtection(RELAY_AIR_ASSIST, false);
            g_airAssistBurstActive = false;
            g_airAssistInOnPhase = false;
        }
        return;
    }

    unsigned long now = millis();

    if (!g_airAssistBurstActive) {
        g_airAssistBurstActive = true;
        g_airAssistInOnPhase = true;
        g_airAssistBurstStart = now;
        applyRelayProtection(RELAY_AIR_ASSIST, true);
        return;
    }

    // Check ON phase: switch to OFF if duration elapsed or millis() wrapped
    if (g_airAssistInOnPhase) {
        if (now < g_airAssistBurstStart ||
            (now - g_airAssistBurstStart >= (g_thresholds.assistOnSec * 1000UL))) {
            g_airAssistInOnPhase = false;
            g_airAssistBurstStart = now;
            applyRelayProtection(RELAY_AIR_ASSIST, false);
        }
    } else {
        // Check OFF phase: switch to ON if duration elapsed or millis() wrapped
        if (now < g_airAssistBurstStart ||
            (now - g_airAssistBurstStart >= (g_thresholds.assistOffSec * 1000UL))) {
            g_airAssistInOnPhase = true;
            g_airAssistBurstStart = now;
            applyRelayProtection(RELAY_AIR_ASSIST, true);
        }
    }
}

// ============================================================
// PID Auto-Tuning
// ============================================================

/**
 * @brief Auto-tune PID gains from the active calibration profile.
 * 
 * Rate-limited: only retunes when confidence changes by >0.05
 * to avoid serial spam if confidence oscillates around the
 * enable threshold.
 * 
 * Called every loop iteration when PID is active, not just on
 * initial enable. This allows gains to update as calibration
 * confidence improves over multiple runs.
 */
static void autoTunePID() {
    BandProfile* profile = adaptive_getActiveProfile();

    if (!profile || !profile->valid) {
        automation_setPidTunings(&g_humidityPID, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
        Serial.println(F("[AUTO] Using default PID gains (no valid profile)"));
        g_lastTunedConfidence = 0.0f;
        return;
    }

    // Rate limit: only retune if confidence changed meaningfully
    if (fabs(profile->confidenceScore - g_lastTunedConfidence) < 0.05f &&
        g_lastTunedConfidence > 0.0f) {
        return;
    }

    if (profile->confidenceScore >= PID_AUTO_ENABLE_CONFIDENCE) {
        pid_tuneFromProfile(&g_humidityPID,
                           profile->avgRecoveryRate,
                           profile->riseTimeSeconds,
                           profile->confidenceScore);
        g_lastTunedConfidence = profile->confidenceScore;

        Serial.print(F("[AUTO] PID tuned: Kp="));
        Serial.print(g_humidityPID.kp, 2);
        Serial.print(F(" Ki="));
        Serial.print(g_humidityPID.ki, 3);
        Serial.print(F(" Kd="));
        Serial.println(g_humidityPID.kd, 2);
    } else {
        automation_setPidTunings(&g_humidityPID, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
        g_lastTunedConfidence = profile->confidenceScore;
        Serial.print(F("[AUTO] Using default gains (confidence="));
        Serial.print(profile->confidenceScore, 2);
        Serial.println(F(" < threshold)"));
    }
}

// ============================================================
// PID Humidity Control
// ============================================================

static void executeHumidityControl() {
    unsigned long now = millis();

    // Atomic read of humidity validity and current value
    float currentRH;
    bool humidityValid;
    portENTER_CRITICAL(&g_stateMux);
    humidityValid = !g_systemState.humiditySensorFault;
    currentRH = g_systemState.currentHumidity;
    portEXIT_CRITICAL(&g_stateMux);

    // Safety: don't run PID if sensor is faulted — suspend and shut down
    if (!humidityValid) {
        if (pid_isEnabled(&g_humidityPID)) {
            suspendPID();
        }
        stopHumidityOutputs();
        return;
    }

    float setpoint = pid_getSetpoint(&g_humidityPID);

    // Hard safety cutoff: at or above ceiling — suspend and shut down
    if (currentRH >= g_thresholds.humCeiling) {
        if (pid_isEnabled(&g_humidityPID)) {
            suspendPID();
        }
        stopHumidityOutputs();
        return;
    }

    // If watchdog tripped, skip PID
    if (g_watchdog.watchdogTripped) return;

    // Compute PID on sample interval
    if (now - g_lastPidCompute < PID_SAMPLE_TIME_MS) return;
    g_lastPidCompute = now;

    float dutyCycle = pid_compute(&g_humidityPID, currentRH);
    float finalDuty = computeDutyCycle(dutyCycle);

    // Boost mode: >5% below setpoint = full ON
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
        }
    }

    updateRelaysFromPID(finalDuty);

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.currentDutyCycle = finalDuty;
    g_systemState.pidOutput = dutyCycle;
    portEXIT_CRITICAL(&g_stateMux);
}

// ============================================================
// Public API: Initialization
// ============================================================

void automation_init() {
    automation_loadDefaults();

    pid_init(&g_humidityPID,
             PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
             PID_SAMPLE_TIME_MS, 0.0f, 100.0f);

    float initialSetpoint = (g_thresholds.humHoHFloor + g_thresholds.humCeiling) / 2.0f;
    pid_setSetpoint(&g_humidityPID, initialSetpoint);
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.setpointHumidity = initialSetpoint;
    portEXIT_CRITICAL(&g_stateMux);

    g_relayTiming.hohLastChange = 0;
    g_relayTiming.assistLastChange = 0;
    g_relayTiming.dutyCycleStartTime = 0;

    g_watchdog.accumulatedOnTime = 0;
    g_watchdog.windowStartTime = 0;
    g_watchdog.lastOnTransitionTime = 0;
    g_watchdog.offDurationStart = 0;
    g_watchdog.watchdogTripped = false;
    g_watchdog.prevRelayState = false;

    g_airAssistBurstActive = false;
    g_airAssistInOnPhase = false;
    g_airAssistBurstStart = 0;

    g_inEmergencyRecovery = false;

    g_lastPidCompute = 0;
    g_boostActive = false;
    g_lastTunedConfidence = 0.0f;

    Serial.println(F("[AUTO] Automation engine initialized"));
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

    Serial.println(F("[AUTO] Loaded default thresholds"));
}

// ============================================================
// Public API: Humidity Control
// ============================================================

void automation_runHumidityLoop() {
    unsigned long now = millis();
    updateWatchdog();

    if (g_systemState.calibrationActive) return;

    // Check manual override — getter handles mutex + auto-expiry
    if (automation_isHumidityOverrideActive()) return;

    // Update active temperature band for telemetry (rate-limited to 5s)
    static unsigned long lastBandUpdate = 0;
    if (now - lastBandUpdate >= 5000UL) {
        lastBandUpdate = now;
        uint8_t band = adaptive_getCurrentBand();
        if (band != g_systemState.currentBand) {
            portENTER_CRITICAL(&g_stateMux);
            g_systemState.currentBand = band;
            portEXIT_CRITICAL(&g_stateMux);
        }
    }

    // Get current humidity with freshness check — all under single critical section
    float currentHumidity;
    {
        bool sensorValid;
        unsigned long faultTimer;
        float liveRH, lastGoodRH;
        portENTER_CRITICAL(&g_stateMux);
        sensorValid = !g_systemState.humiditySensorFault;
        liveRH = g_systemState.currentHumidity;
        lastGoodRH = g_systemState.lastKnownGoodHumidity;
        faultTimer = g_systemState.sensorFaultTimer;
        portEXIT_CRITICAL(&g_stateMux);

        if (sensorValid) {
            currentHumidity = liveRH;
        } else if (faultTimer > 0 && (millis() - faultTimer < 30000UL)) {
            // Fault is recent — use last known good value
            currentHumidity = lastGoodRH;
        } else {
            // Sensor faulted >30s or never had valid data — safe mode
            if (pid_isEnabled(&g_humidityPID)) suspendPID();
            stopHumidityOutputs();
            return;
        }
    }

    // ================================================================
    // CEILING CHECK: At or above ceiling, everything OFF
    // ================================================================
    if (currentHumidity >= g_thresholds.humCeiling) {
        if (pid_isEnabled(&g_humidityPID)) {
            suspendPID();
        }
        stopHumidityOutputs();
        g_inEmergencyRecovery = false;
        return;
    }

    // ================================================================
    // BELOW ASSIST FLOOR: Emergency recovery
    // HOH ON full, Air Assist bursting — bypass PID entirely
    // ================================================================
    if (currentHumidity < g_thresholds.humAssistFloor) {
        // Watchdog check: if HOH is locked out, do NOT re-enable it
        if (g_watchdog.watchdogTripped) {
            return;
        }

        if (!g_inEmergencyRecovery) {
            if (pid_isEnabled(&g_humidityPID)) {
                suspendPID();
            }
            g_inEmergencyRecovery = true;
            Serial.println(F("[AUTO] Entering emergency recovery mode"));
        }

        // Update telemetry to reflect actual actuator state (100% ON)
        portENTER_CRITICAL(&g_stateMux);
        g_systemState.currentDutyCycle = 100.0f;
        g_systemState.pidOutput = 0.0f;
        portEXIT_CRITICAL(&g_stateMux);

        applyRelayProtection(RELAY_HOH, true);
        runAirAssistBursts();
        return;
    }

    // ================================================================
    // BETWEEN ASSIST FLOOR AND HOH FLOOR: HOH only (no Air Assist)
    // ================================================================
    if (currentHumidity < g_thresholds.humHoHFloor) {
        g_inEmergencyRecovery = false;

        // Update telemetry to reflect actual actuator state (100% ON)
        portENTER_CRITICAL(&g_stateMux);
        g_systemState.currentDutyCycle = 100.0f;
        g_systemState.pidOutput = 0.0f;
        portEXIT_CRITICAL(&g_stateMux);

        applyRelayProtection(RELAY_HOH, true);
        if (g_airAssistBurstActive || relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
            applyRelayProtection(RELAY_AIR_ASSIST, false);
            g_airAssistBurstActive = false;
            g_airAssistInOnPhase = false;
        }
        return;
    }

    // ================================================================
    // MAINTENANCE ZONE: Between HOH floor and ceiling
    // Use PID if confident, otherwise bang-bang OFF
    // ================================================================
    g_inEmergencyRecovery = false;

    BandProfile* profile = adaptive_getActiveProfile();
    bool pidReady = (profile && profile->valid &&
                     profile->confidenceScore >= PID_AUTO_ENABLE_CONFIDENCE &&
                     !g_watchdog.watchdogTripped);

    if (pidReady) {
        if (!pid_isEnabled(&g_humidityPID)) {
            pid_setEnabled(&g_humidityPID, true);
            float newSetpoint = (g_thresholds.humHoHFloor + g_thresholds.humCeiling) / 2.0f;
            pid_setSetpoint(&g_humidityPID, newSetpoint);
            portENTER_CRITICAL(&g_stateMux);
            g_systemState.setpointHumidity = newSetpoint;
            portEXIT_CRITICAL(&g_stateMux);
        }
        // Called every loop when PID active — internally rate-limited
        autoTunePID();
        executeHumidityControl();
    } else {
        if (pid_isEnabled(&g_humidityPID)) {
            suspendPID();
        }
        applyRelayProtection(RELAY_HOH, false);
        if (g_airAssistBurstActive || relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
            applyRelayProtection(RELAY_AIR_ASSIST, false);
            g_airAssistBurstActive = false;
            g_airAssistInOnPhase = false;
        }
    }
}

// ============================================================
// Public API: CO2 Control
// ============================================================

void automation_runCO2Loop() {
    if (g_systemState.calibrationActive) return;

    // Check manual override — getter handles mutex + auto-expiry
    if (automation_isCO2OverrideActive()) return;

    uint16_t currentCO2 = sensors_isCO2Valid() ?
        g_systemState.currentCO2 : g_systemState.lastKnownGoodCO2;

    if (currentCO2 > g_thresholds.co2HighLimit) {
        if (!relayManager_isRelayOn(RELAY_EXHAUST)) {
            relayManager_setRelay(RELAY_EXHAUST, true);
        }
    }

    if (currentCO2 <= g_thresholds.co2LowTarget) {
        if (relayManager_isRelayOn(RELAY_EXHAUST)) {
            relayManager_setRelay(RELAY_EXHAUST, false);
        }
    }

    static bool emergencyAlertSent = false;
    if (currentCO2 >= g_thresholds.co2Emergency) {
        if (!emergencyAlertSent) {
            emergencyAlertSent = true;
            char msg[48];
            snprintf(msg, sizeof(msg), "CO2 level: %u ppm. Check ventilation!", currentCO2);
            network_sendAlert("CO2 Emergency", msg);
        }
    } else if (currentCO2 < g_thresholds.co2Emergency - 50) {
        emergencyAlertSent = false;
    }
}

// ============================================================
// Public API: Night Mode
// ============================================================

void automation_checkNightMode() {
    bool nightMode = rtc_isNightMode();

    if (nightMode != g_systemState.nightModeActive) {
        portENTER_CRITICAL(&g_stateMux);
        g_systemState.nightModeActive = nightMode;
        portEXIT_CRITICAL(&g_stateMux);

        if (nightMode) {
            Serial.println(F("[AUTO] Night Mode ACTIVATED (21:00-10:00)"));

            // Lock out compressor and Air Assist per noise ordinance.
            // HOH is silent — allowed to continue operating at night.
            // Compressor cooldown is enforced by relay_manager internally;
            // turning OFF is always safe (cooldown only prevents re-start).
            relayManager_setRelay(RELAY_COMPRESSOR, false);
            if (relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
                relayManager_setRelay(RELAY_AIR_ASSIST, false);
            }

            // Reset burst state so morning starts clean
            g_airAssistBurstActive = false;
            g_airAssistInOnPhase = false;
            g_airAssistBurstStart = 0;

            // Abort any running calibration immediately.
            // Starts are prevented within 30 minutes of night mode
            // by rtc_minutesUntilNightMode() in adaptive.cpp.
            if (adaptive_isCalibrating()) {
                adaptive_cancelCalibration();
                Serial.println(F("[AUTO] Calibration aborted — night mode active"));
            }
        } else {
            Serial.println(F("[AUTO] Night Mode DEACTIVATED"));
        }
    }

    // Continuous enforcement during night mode
    if (nightMode) {
        if (relayManager_isRelayOn(RELAY_COMPRESSOR)) {
            relayManager_setRelay(RELAY_COMPRESSOR, false);
        }
        if (relayManager_isRelayOn(RELAY_AIR_ASSIST)) {
            relayManager_setRelay(RELAY_AIR_ASSIST, false);
            g_airAssistBurstActive = false;
            g_airAssistInOnPhase = false;
        }
    }
}

// ============================================================
// Public API: Temperature Alerts
// ============================================================

void automation_checkTemperatureAlerts() {
    float currentTemp = sensors_isTemperatureValid() ?
        g_systemState.currentTemp : g_systemState.lastKnownGoodTemp;

    static bool highAlertSent = false;
    static bool lowAlertSent = false;
    static unsigned long lastHighAlertTime = 0;
    static unsigned long lastLowAlertTime = 0;

    // High temperature alert with 5-minute cooldown between alerts.
    // lastHighAlertTime==0 allows first alert to fire immediately.
    if (currentTemp >= TEMP_ALERT_HIGH_C) {
        if (!highAlertSent && (lastHighAlertTime == 0 || millis() - lastHighAlertTime >= 300000UL)) {
            highAlertSent = true;
            lastHighAlertTime = millis();
            char msg[48];
            snprintf(msg, sizeof(msg), "Temperature: %.1f C", currentTemp);
            network_sendAlert("High Temperature", msg);
        }
    } else if (currentTemp < TEMP_ALERT_HIGH_C - 1.0f) {
        highAlertSent = false;
    }

    // Low temperature alert with 5-minute cooldown between alerts.
    // lastLowAlertTime==0 allows first alert to fire immediately.
    if (currentTemp <= TEMP_ALERT_LOW_C) {
        if (!lowAlertSent && (lastLowAlertTime == 0 || millis() - lastLowAlertTime >= 300000UL)) {
            lowAlertSent = true;
            lastLowAlertTime = millis();
            char msg[48];
            snprintf(msg, sizeof(msg), "Temperature: %.1f C", currentTemp);
            network_sendAlert("Low Temperature", msg);
        }
    } else if (currentTemp > TEMP_ALERT_LOW_C + 1.0f) {
        lowAlertSent = false;
    }
}

// ============================================================
// Public API: Threshold Management
// ============================================================

AutomationThresholds* automation_getThresholds() {
    return &g_thresholds;
}

void automation_updateThresholds(const AutomationThresholds* newThresholds) {
    if (!newThresholds) return;

    if (newThresholds->humHoHFloor < 0 || newThresholds->humHoHFloor > 100) return;
    if (newThresholds->humAssistFloor < 0 || newThresholds->humAssistFloor > 100) return;
    if (newThresholds->humCeiling < 0 || newThresholds->humCeiling > 100) return;
    if (newThresholds->co2HighLimit < 400 || newThresholds->co2HighLimit > 5000) return;
    if (newThresholds->co2LowTarget < 400 || newThresholds->co2LowTarget > 5000) return;
    if (newThresholds->co2Emergency < 400 || newThresholds->co2Emergency > 5000) return;
    if (newThresholds->humAssistFloor > newThresholds->humHoHFloor) return;
    if (newThresholds->humHoHFloor >= newThresholds->humCeiling) return;
    // Validate ordering before hysteresis check
    if (newThresholds->co2LowTarget >= newThresholds->co2HighLimit) return;
    // CO2 hysteresis: require at least 100ppm gap between high and low thresholds
    if (newThresholds->co2HighLimit - newThresholds->co2LowTarget < 100) return;
    if (newThresholds->co2HighLimit >= newThresholds->co2Emergency) return;
    if (newThresholds->assistOnSec < 1 || newThresholds->assistOnSec > 60) return;
    if (newThresholds->assistOffSec < 1 || newThresholds->assistOffSec > 300) return;
    // Prevent Air Assist running near-continuously (ON must be <= OFF)
    if (newThresholds->assistOnSec > newThresholds->assistOffSec) return;

    // Update thresholds under mutex — Web UI may read concurrently
    portENTER_CRITICAL(&g_stateMux);
    memcpy(&g_thresholds, newThresholds, sizeof(AutomationThresholds));
    portEXIT_CRITICAL(&g_stateMux);

    Serial.println(F("[AUTO] Thresholds updated and validated"));
}

bool automation_isAirAssistBurstActive() {
    return g_airAssistBurstActive;
}

// ============================================================
// Public API: Manual Overrides
// ============================================================

void automation_activateHumidityOverride() {
    portENTER_CRITICAL(&g_stateMux);
    g_humidityOverrideActive = true;
    g_humidityOverrideStart = millis();
    portEXIT_CRITICAL(&g_stateMux);

    if (pid_isEnabled(&g_humidityPID)) {
        suspendPID();
    }
    // Force all humidity outputs OFF so manual control starts from known state
    stopHumidityOutputs();
    // Clear telemetry for dashboard (suspendPID wasn't called if PID was already off)
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.currentDutyCycle = 0.0f;
    g_systemState.pidOutput = 0.0f;
    portEXIT_CRITICAL(&g_stateMux);
    Serial.println(F("[AUTO] Humidity override ACTIVATED"));
}

void automation_activateCO2Override() {
    portENTER_CRITICAL(&g_stateMux);
    g_co2OverrideActive = true;
    g_co2OverrideStart = millis();
    portEXIT_CRITICAL(&g_stateMux);
    Serial.println(F("[AUTO] CO2 override ACTIVATED"));
}

void automation_deactivateHumidityOverride() {
    portENTER_CRITICAL(&g_stateMux);
    g_humidityOverrideActive = false;
    portEXIT_CRITICAL(&g_stateMux);
    g_lastPidCompute = 0;  // Force immediate PID computation on resume
    Serial.println(F("[AUTO] Humidity override DEACTIVATED"));
}

void automation_deactivateCO2Override() {
    portENTER_CRITICAL(&g_stateMux);
    g_co2OverrideActive = false;
    portEXIT_CRITICAL(&g_stateMux);
    Serial.println(F("[AUTO] CO2 override DEACTIVATED"));
}

void automation_deactivateAllOverrides() {
    portENTER_CRITICAL(&g_stateMux);
    g_humidityOverrideActive = false;
    g_co2OverrideActive = false;
    portEXIT_CRITICAL(&g_stateMux);
    g_lastPidCompute = 0;  // Force immediate PID computation on resume
    Serial.println(F("[AUTO] All overrides DEACTIVATED"));
}

bool automation_isHumidityOverrideActive() {
    bool active;
    unsigned long start;
    portENTER_CRITICAL(&g_stateMux);
    active = g_humidityOverrideActive;
    start = g_humidityOverrideStart;
    portEXIT_CRITICAL(&g_stateMux);

    if (active && (millis() - start >= MANUAL_OVERRIDE_TIMEOUT_MS)) {
        portENTER_CRITICAL(&g_stateMux);
        g_humidityOverrideActive = false;
        portEXIT_CRITICAL(&g_stateMux);
        Serial.println(F("[AUTO] Humidity override TIMEOUT"));
        return false;
    }
    return active;
}

bool automation_isCO2OverrideActive() {
    bool active;
    unsigned long start;
    portENTER_CRITICAL(&g_stateMux);
    active = g_co2OverrideActive;
    start = g_co2OverrideStart;
    portEXIT_CRITICAL(&g_stateMux);

    if (active && (millis() - start >= MANUAL_OVERRIDE_TIMEOUT_MS)) {
        portENTER_CRITICAL(&g_stateMux);
        g_co2OverrideActive = false;
        portEXIT_CRITICAL(&g_stateMux);
        Serial.println(F("[AUTO] CO2 override TIMEOUT"));
        return false;
    }
    return active;
}

unsigned long automation_getHumidityOverrideRemaining() {
    bool active;
    unsigned long start;
    portENTER_CRITICAL(&g_stateMux);
    active = g_humidityOverrideActive;
    start = g_humidityOverrideStart;
    portEXIT_CRITICAL(&g_stateMux);

    if (!active) return 0;
    unsigned long elapsed = millis() - start;
    if (elapsed >= MANUAL_OVERRIDE_TIMEOUT_MS) return 0;
    return MANUAL_OVERRIDE_TIMEOUT_MS - elapsed;
}

unsigned long automation_getCO2OverrideRemaining() {
    bool active;
    unsigned long start;
    portENTER_CRITICAL(&g_stateMux);
    active = g_co2OverrideActive;
    start = g_co2OverrideStart;
    portEXIT_CRITICAL(&g_stateMux);

    if (!active) return 0;
    unsigned long elapsed = millis() - start;
    if (elapsed >= MANUAL_OVERRIDE_TIMEOUT_MS) return 0;
    return MANUAL_OVERRIDE_TIMEOUT_MS - elapsed;
}
