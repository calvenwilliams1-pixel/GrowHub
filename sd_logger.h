/*
   sd_logger.h
   GrowHub32 - MicroSD Data Logging & Configuration Storage
   Version: 1.2.3
   Revision: Added compressor cooldown fields to RuntimeCache for persistence
             across reboots (GH-SAFE-002 persistent).
             Uses rtc_getEpochSeconds() for consistent timestamp handling.
*/

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include "config.h"
#include "automation.h"

// Log entry structure for daily CSV
struct LogEntry {
  char timestamp[20];
  float temperature;
  float humidity;
  uint16_t co2;
  float fridgeTemp;
  bool hoHActive;
  bool airAssistActive;
  bool exhaustFanActive;
  bool compressorActive;
  bool nightMode;
};

// Daily summary statistics for archive
struct DailySummary {
  char date[11];        // YYYY-MM-DD
  float tempMin;
  float tempMax;
  float tempAvg;
  float humMin;
  float humMax;
  float humAvg;
  uint16_t co2Min;
  uint16_t co2Max;
  uint16_t co2Avg;
};

// Configuration cache structure (persisted to SD for reboot recovery)
struct RuntimeCache {
  AutomationThresholds thresholds;
  float emaWeight;
  unsigned long totalRuntimeHours;
  uint8_t lastActiveBand;

  // GH-SAFE-002 persistent: Compressor cooldown state across reboots
  // Prevents compressor restart when still thermally hot after power loss
  unsigned long compressorLastOffTimestamp;  // RTC epoch seconds when compressor was last turned off
  bool compressorCooldownActive;             // Whether cooldown was active at last save
};

extern RuntimeCache g_runtimeCache;

// Public API
bool sdLogger_init();
bool sdLogger_isAvailable();
bool sdLogger_writeData();
bool sdLogger_loadCache();
bool sdLogger_saveCache();
bool sdLogger_saveCooldownState();
bool sdLogger_loadCooldownState();
bool sdLogger_purgeOldLogs();
bool sdLogger_checkFileIntegrity(const char* path);
void sdLogger_logSystemEvent(const char* event);

// Daily log file management
String sdLogger_getCurrentLogFilename();
bool sdLogger_writeHeader(const char* filename);

#endif // SD_LOGGER_H
