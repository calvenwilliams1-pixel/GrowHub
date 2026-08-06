/*
   sd_logger.h
   GrowHub32 - MicroSD Data Logging & Configuration Storage
   Version: 1.4.0
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
#include "relay_manager.h"

// Log entry structure for daily CSV
struct LogEntry {
  char timestamp[20];
  float temperature;
  float humidity;
  uint16_t co2;
  float fridgeTemp;
  float fridgeHumidity;
  bool fridgeDoorOpen;
  bool hoHActive;
  bool airAssistActive;
  bool exhaustFanActive;
  bool compressorActive;
  bool nightMode;
};

// ============================================================
// Graph Dashboard — Historical Data Query (v1.4)
// ============================================================
// Timestamps are Unix epoch (seconds since 1970-01-01) for JS compatibility.

struct GraphDataRequest {
  uint32_t startEpoch;      // Unix epoch — start of range
  uint32_t endEpoch;        // Unix epoch — end of range (0 = now)
  uint8_t  sensorType;      // 0=temp, 1=humidity, 2=CO2, 3=fridge
  uint16_t maxPoints;       // Max points requested (server caps at GRAPH_MAX_RESPONSE_POINTS)
  uint32_t requestId;       // Client sequence number for race-condition prevention
};

// SD card mutex — protects all file operations across tasks
extern SemaphoreHandle_t g_sdMutex;
bool sdLogger_initMutex();

// Returns JSON string length written to output buffer.
// Output: {"type":100,"s":<sensor>,"rid":<id>,"p":[[t1,v1],...]}
// On error/no data: {"type":100,"s":<sensor>,"rid":<id>,"p":[]}
size_t sdLogger_getHistoricalData(const GraphDataRequest* request, char* output, size_t outputMax);

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
  uint8_t version;           // Cache format version (2 = v1.5+)
  AutomationThresholds thresholds;
  float emaWeight;
  unsigned long totalRuntimeHours;
  uint8_t lastActiveBand;

  // GH-SAFE-002 persistent: Compressor cooldown state across reboots
  unsigned long compressorLastOffTimestamp;
  bool compressorCooldownActive;

  // v1.4: Configurable relay mapping persistence
  RelayMapping relayMapping;
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
