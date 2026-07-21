/*
   adaptive.h
   GrowHub32 - Adaptive Learning Mechanics
   Version: 1.3
   Revision: Added active calibration phase state machine (v1.3).
             Calibration now actively controls HOH + Air Assist through
             STABILIZE → RISE → DECAY → COMPLETE phases instead of
             passively watching ambient RH drift.

   Handles:
   - Temperature band profile management (GH-AL-001, GH-AL-002)
   - Dynamic band selection based on live temperature (GH-AL-003)
   - Active calibration sequence execution (GH-AL-004)
   - EMA smoothing of historical vs new data (GH-AL-005)
   - Confidence score computation (GH-AL-006)
   - Recovery time projections for simulation UI
*/

#ifndef ADAPTIVE_H
#define ADAPTIVE_H

#include <Arduino.h>
#include "config.h"

// NOTE: Keep this struct POD-only (plain old data: floats, ints, bools).
// The copy assignment operator is used in readProfileJSON() for atomic updates.
// Adding Arduino String objects or raw pointers will break shallow-copy safety.
struct BandProfile {
  float riseTimeSeconds;      // Time to rise from floor to ceiling
  float decayTimeSeconds;     // Time to decay from ceiling to floor
  float avgRecoveryRate;      // Average RH % recovery per second
  float confidenceScore;      // 0.0 to 0.90 (capped per GH-AL-006)
  uint32_t sampleCount;       // Number of calibration samples collected
  bool valid;                 // Whether profile has been calibrated
};

// Calibration phase tracking (v1.3 active calibration)
enum CalibrationPhase {
  CALIB_PHASE_STABILIZE = 0,  // All actuators OFF, waiting for RH to settle
  CALIB_PHASE_RISE,           // HOH + Air Assist ON, measuring recovery rate
  CALIB_PHASE_DECAY,          // All actuators OFF, measuring natural decay
  CALIB_PHASE_COMPLETE        // Results calculated and saved
};

// Current active band tracking
struct AdaptiveState {
  uint8_t activeBandIndex;        // Currently active band (0-3)
  uint8_t calibrationBand;        // Band locked at calibration start (prevents drift during calibration window)
  bool calibrationActive;         // Calibration in progress flag
  unsigned long calibrationStartTime; // millis() when calibration started
  float calibrationStartRH;       // RH at calibration start
  float calibrationPeakRH;        // Highest RH observed during calibration
  float calibrationLowRH;         // Lowest RH observed during calibration
  float emaWeight;               // Current EMA weight (0.10 to 0.50)

  // v1.3: Active calibration phase tracking
  CalibrationPhase calibPhase;
  unsigned long calibPhaseStart;         // millis() when current phase started
  float calibPhaseBaselineRH;            // RH at start of rise phase (for delta calculation)
  bool calibPhasePlateauDetected;        // Rise phase plateau flag
  unsigned long calibPlateauSampleStart; // Start of plateau slope window
  float calibPlateauSampleRH;            // RH at start of plateau slope window
};

extern AdaptiveState g_adaptiveState;

// Public API
bool adaptive_init();
bool adaptive_loadProfiles();
bool adaptive_saveProfile(uint8_t bandIndex);
uint8_t adaptive_getCurrentBand();
BandProfile* adaptive_getActiveProfile();

// Calibration
bool adaptive_startCalibration();
void adaptive_updateCalibration();
bool adaptive_isCalibrating();
void adaptive_cancelCalibration();
unsigned long adaptive_getCalibrationStartTime();

// EMA smoothing (GH-AL-005)
float adaptive_applyEMA(float rawValue, float storedHistorical);

// Confidence score update (GH-AL-006)
float adaptive_updateConfidence(uint8_t bandIndex);

// Recovery projection (GH-UI-004)
float adaptive_projectRecoveryTime(float deltaToTarget);

// Weight management
void adaptive_setEMAWeight(float weight);
float adaptive_getEMAWeight();

#endif // ADAPTIVE_H