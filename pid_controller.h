/*
   pid_controller.h
   GrowHub32 - PID Controller for Humidity Regulation
   Version: 1.3.3
   Revision: Added initialized flag for proper reset/preserve behavior.
             integralMin defaults to 0.0f for unidirectional humidifier control.
             Added config.h include for PID_DEFAULT_KP macro.

   Standard discrete PID controller with:
   - Integral anti-windup (conditional accumulation)
   - Derivative-on-measurement (prevents derivative kick on setpoint change)
   - Auto-tuning from calibration BandProfile data with gain clamping
   - Bumpless transfer (integral reset on gain changes)
   - lastOutput caching for consistent output between sample intervals
   - Derivative low-pass filtering for sensor noise rejection
   - Initialized flag distinguishes fresh start from post-reset state
*/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

typedef struct {
  // User-tunable gains
  float kp;              // Proportional gain
  float ki;              // Integral gain
  float kd;              // Derivative gain

  // Configuration
  unsigned long sampleTimeMs;  // PID recompute interval
  float outputMin;             // Minimum output (0.0 = 0% duty)
  float outputMax;             // Maximum output (100.0 = 100% duty)
  float integralMax;           // Anti-windup clamp ceiling
  float integralMin;           // Anti-windup clamp floor (0.0 for unidirectional)

  // Internal state
  bool initialized;            // Distinguishes fresh start from post-reset state
  float setpoint;              // Target value
  float lastInput;             // Previous measurement (for derivative-on-measurement)
  float lastDerivative;        // Low-pass filtered derivative value
  float integral;              // Accumulated error
  float lastOutput;            // Last computed output (returned on off-cycle calls)
  unsigned long lastUpdateTime; // Last millis() when compute() ran
  bool enabled;                // Controller active flag
} PIDController;

// Initialize a PID controller with gains and limits
void pid_init(PIDController* pid, float kp, float ki, float kd,
              unsigned long sampleTimeMs, float outputMin, float outputMax);

// Set the target setpoint
void pid_setSetpoint(PIDController* pid, float sp);

// Compute PID output. Call at regular intervals (sampleTimeMs).
// Between sample intervals, returns the last computed output (not 0).
// Returns duty cycle percentage (0.0–100.0).
float pid_compute(PIDController* pid, float input);

// Auto-tune gains from calibration profile data.
// recoveryRate = % RH per second, riseTime = seconds, confidence = 0.0–0.90
// Gains are clamped to safe ranges for humidity control.
void pid_tuneFromProfile(PIDController* pid, float recoveryRate,
                          float riseTime, float confidence);

// Reset internal state (integral, derivative, last output, timer).
// Preserves lastInput and initialized flag for smooth post-reset transition.
void pid_reset(PIDController* pid);

// Enable or disable the controller
void pid_setEnabled(PIDController* pid, bool enable);

// Check if controller is enabled
bool pid_isEnabled(PIDController* pid);

// Get current setpoint
float pid_getSetpoint(PIDController* pid);

#endif // PID_CONTROLLER_H