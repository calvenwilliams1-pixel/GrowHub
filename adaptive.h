/*
   adaptive.h
   GrowHub32 - Adaptive Learning Mechanics
   Version: 1.2.1

   Handles:
   - Temperature band profile management (GH-AL-001, GH-AL-002)
   - Dynamic band selection based on live temperature (GH-AL-003)
   - Calibration sequence execution (GH-AL-004)
   - EMA smoothing of historical vs new data (GH-AL-005)
   - Confidence score computation (GH-AL-006)
   - Recovery time projections for simulation UI
*/

#ifndef ADAPTIVE_H
#define ADAPTIVE_H

#include <Arduino.h>
#include "config.h"

// Temperature band profile structure (GH-AL-002)
struct BandProfile {
  float riseTimeSeconds;      // Time to rise from floor to ceiling
  float decayTimeSeconds;     // Time to decay from ceiling to floor
  float avgRecoveryRate;      // Average RH % recovery per second
  float confidenceScore;      // 0.0 to 0.90 (capped per GH-AL-006)
  uint32_t sampleCount;       // Number of calibration samples collected
  bool valid;                 // Whether profile has been calibrated
};

// Current active band tracking
struct AdaptiveState {
  uint8_t activeBandIndex;    // 0=18-21, 1=21-24, 2=24-27, 3=27-30
  bool calibrationActive;
  unsigned long calibrationStartTime;
  float calibrationStartRH;
  float calibrationPeakRH;
  float calibrationLowRH;
  float emaWeight;            // Current EMA weight (0.10 to 0.50)
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

// Band boundary definitions
#define BAND_18_21_INDEX  0
#define BAND_21_24_INDEX  1
#define BAND_24_27_INDEX  2
#define BAND_27_30_INDEX  3

#define BAND_18_21_LOW    18.0f
#define BAND_18_21_HIGH   21.0f
#define BAND_21_24_LOW    21.0f
#define BAND_21_24_HIGH   24.0f
#define BAND_24_27_LOW    24.0f
#define BAND_24_27_HIGH   27.0f
#define BAND_27_30_LOW    27.0f
#define BAND_27_30_HIGH   30.0f

#endif // ADAPTIVE_H
