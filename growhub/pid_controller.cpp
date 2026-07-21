/*
   pid_controller.cpp
   GrowHub32 - PID Controller Implementation
   Version: 1.3.3
   Revision: Rescaled Ki/Kd auto-tuning formulas to minutes (prevents clamp saturation).
             Added initialized flag for correct first-call behavior after reset.
             Fixed first-sample 0% duty drop (now computes immediately).
             Added conditional integral accumulation to prevent windup on overshoot.
             Added gain clamping after auto-tuning (safe ranges for humidity).
             Added derivative low-pass filtering for SCD40 sensor noise.
             integralMin defaults to 0.0f for unidirectional humidifier control.
*/

#include "pid_controller.h"

void pid_init(PIDController* pid, float kp, float ki, float kd,
              unsigned long sampleTimeMs, float outputMin, float outputMax) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->sampleTimeMs = sampleTimeMs;
  pid->outputMin = outputMin;
  pid->outputMax = outputMax;

  // Integral clamp: 0.0 floor for unidirectional humidifier.
  // Humidity can only be added, never actively removed. A negative integral
  // accumulated during overshoot would delay recovery when RH drops back down.
  float range = outputMax - outputMin;
  pid->integralMax = range * 0.3f;
  pid->integralMin = 0.0f;

  pid->initialized = false;
  pid->setpoint = 0.0f;
  pid->lastInput = 0.0f;
  pid->lastDerivative = 0.0f;
  pid->integral = 0.0f;
  pid->lastOutput = 0.0f;
  pid->lastUpdateTime = 0;
  pid->enabled = true;
}

void pid_setSetpoint(PIDController* pid, float sp) {
  pid->setpoint = sp;
}

float pid_getSetpoint(PIDController* pid) {
  return pid->setpoint;
}

float pid_compute(PIDController* pid, float input) {
  if (!pid->enabled) return 0.0f;

  unsigned long now = millis();

  // On first call (fresh start or after reset), initialize timing.
  // If this is a post-reset call and lastInput was preserved from before
  // the reset, use it to avoid a derivative spike from (input - 0) / dt.
  if (pid->lastUpdateTime == 0) {
    pid->lastUpdateTime = now - pid->sampleTimeMs;

    if (!pid->initialized) {
      // Fresh start — no previous data, seed lastInput with current value
      pid->lastInput = input;
      pid->initialized = true;
    }
    // If initialized is true, this is a post-reset call and lastInput
    // was preserved — skip the seed so derivative sees (input - lastInput)
    // rather than (input - 0), which would produce a large spike.
  }

  // Return cached output between sample intervals for consistent duty cycle
  if (now - pid->lastUpdateTime < pid->sampleTimeMs) {
    return pid->lastOutput;
  }

  float dt = (now - pid->lastUpdateTime) / 1000.0f;
  pid->lastUpdateTime = now;

  // Guard against zero or negative dt (clock wraparound, scheduler jitter)
  if (dt <= 0.0f) return pid->lastOutput;

  // Calculate error
  float error = pid->setpoint - input;

  // --- Proportional term ---
  float pTerm = pid->kp * error;

  // --- Integral term with conditional accumulation ---
  // Only grow the integral if it helps move the output back toward the valid
  // range. This prevents deep negative integral accumulation during overshoot
  // that would delay recovery when RH drops back below target.
  float newIntegral = pid->integral + (error * dt);
  float unclampedOutput = pTerm + (pid->ki * newIntegral) - (pid->kd * pid->lastDerivative);

  bool integralCanGrow = false;
  if (unclampedOutput >= pid->outputMin && unclampedOutput <= pid->outputMax) {
    integralCanGrow = true;
  } else if (unclampedOutput > pid->outputMax && error < 0.0f) {
    integralCanGrow = true;
  } else if (unclampedOutput < pid->outputMin && error > 0.0f) {
    integralCanGrow = true;
  }

  if (integralCanGrow) {
    pid->integral = newIntegral;
  }

  // Clamp integral to anti-windup limits
  if (pid->integral > pid->integralMax) {
    pid->integral = pid->integralMax;
  } else if (pid->integral < pid->integralMin) {
    pid->integral = pid->integralMin;
  }

  float iTerm = pid->ki * pid->integral;

  // --- Derivative term (on measurement, with low-pass filter) ---
  // Derivative-on-measurement prevents "derivative kick" on setpoint changes.
  // Low-pass filter (80% previous, 20% new) rejects SCD40 sensor noise
  // that would otherwise create false derivative spikes.
  float rawDerivative = (input - pid->lastInput) / dt;
  pid->lastDerivative = (0.8f * pid->lastDerivative) + (0.2f * rawDerivative);
  float dTerm = -pid->kd * pid->lastDerivative;

  pid->lastInput = input;

  // --- Compute and clamp output ---
  float output = pTerm + iTerm + dTerm;

  if (output > pid->outputMax) output = pid->outputMax;
  if (output < pid->outputMin) output = pid->outputMin;

  // Cache for off-cycle calls
  pid->lastOutput = output;

  return output;
}

void pid_tuneFromProfile(PIDController* pid, float recoveryRate,
                          float riseTime, float confidence) {
  // Scale gains by confidence — conservative when confidence is low.
  // Scale range: 0.3 (very conservative) to 1.0 (fully tuned).
  float scale = 0.3f + (confidence * 0.7f);

  // Proportional gain: inversely proportional to recovery rate.
  // Faster recovery = more sensitive system = lower gain needed.
  if (recoveryRate > 0.001f) {
    pid->kp = (1.0f / recoveryRate) * 0.5f * scale;
  } else {
    pid->kp = PID_DEFAULT_KP * scale;
  }

  // Integral gain: based on rise time converted to minutes.
  // Dividing by 60 keeps realistic values (5-10 min rise times) inside
  // the clamp range rather than saturating at 0.05 for any riseTime > 5s.
  pid->ki = (riseTime / 60.0f) * 0.01f * scale;

  // Derivative gain: same rescale to minutes.
  pid->kd = (riseTime / 60.0f) * 0.005f * scale;

  // Clamp gains to safe ranges for humidity control.
  // These bounds prevent auto-tuning from producing unreasonably aggressive
  // values that would cause oscillation or integral windup on real hardware.
  if (pid->kp < 0.1f) pid->kp = 0.1f;
  if (pid->kp > 3.0f) pid->kp = 3.0f;
  if (pid->ki < 0.0f) pid->ki = 0.0f;
  if (pid->ki > 0.05f) pid->ki = 0.05f;
  if (pid->kd < 0.0f) pid->kd = 0.0f;
  if (pid->kd > 1.0f) pid->kd = 1.0f;

  // Reset integral accumulator to prevent windup from old gains
  pid->integral = 0.0f;
}

void pid_reset(PIDController* pid) {
  // Reset integral, derivative filter, and output cache.
  // Preserve lastInput and initialized flag — the physical sensor value
  // hasn't changed just because we reset the controller (e.g., on band
  // switch). On the next compute() call, the first-call logic will use
  // the preserved lastInput to avoid a derivative spike from (input - 0).
  pid->integral = 0.0f;
  pid->lastDerivative = 0.0f;
  pid->lastOutput = 0.0f;
  pid->lastUpdateTime = 0;
}

void pid_setEnabled(PIDController* pid, bool enable) {
  pid->enabled = enable;
  if (!enable) {
    pid_reset(pid);
  }
}

bool pid_isEnabled(PIDController* pid) {
  return pid->enabled;
}