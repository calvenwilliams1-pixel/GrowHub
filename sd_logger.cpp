/*
   sd_logger.cpp
   GrowHub32 - MicroSD Data Logging Implementation
   Version: 1.2.3
   Revision: Added compressor cooldown persistence in RuntimeCache.
             Cooldown state saved on every log write and on compressor state changes.
             Uses rtc_getEpochSeconds() for consistent timestamp handling.
             Cache validation improved with range checking.

   SD Card connections (SPI):
   - CS:  GPIO 5
   - SCK: GPIO 18
   - MOSI: GPIO 23
   - MISO: GPIO 19
*/

#include "sd_logger.h"
#include "rtc_handler.h"
#include "sensors.h"
#include "relay_manager.h"
#include "automation.h"
#include "network.h"

// External declarations
extern AutomationThresholds* automation_getThresholds();
extern bool relayManager_isCompressorCooldownActive();

RuntimeCache g_runtimeCache;

static bool g_sdAvailable = false;
static SPIClass g_sdSPI(VSPI);
static String g_currentLogFile = "";

// Track compressor state changes to trigger cooldown saves
static bool g_lastCompressorState = false;

// --- Private Helpers ---

static String buildDateFilename() {
  RTCTime now;
  if (!rtc_readTime(&now)) {
    // Fallback: use compile date if RTC unavailable
    return "/logs/log_unknown_date.csv";
  }

  char filename[40];
  snprintf(filename, sizeof(filename), "/logs/log_%04d-%02d-%02d.csv",
           now.year, now.month, now.date);
  return String(filename);
}

static bool fileExists(const char* path) {
  File f = SD.open(path);
  if (f) {
    f.close();
    return true;
  }
  return false;
}

static bool writeLine(const char* filename, const String& line) {
  File f = SD.open(filename, FILE_APPEND);
  if (!f) {
    Serial.print(F("[SD] Failed to open file for append: "));
    Serial.println(filename);
    return false;
  }

  size_t written = f.println(line);
  f.close();

  if (written == 0) {
    Serial.println(F("[SD] Failed to write line - card may be full or corrupted"));
    return false;
  }

  return true;
}

// --- Public API ---

bool sdLogger_init() {
  Serial.println(F("[SD] Initializing MicroSD card..."));

  // Initialize SPI bus for SD card
  g_sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Try to mount SD card
  if (!SD.begin(SD_CS_PIN, g_sdSPI)) {
    Serial.println(F("[SD] Card mount FAILED!"));
    Serial.println(F("[SD] Check:"));
    Serial.println(F("  - Card is formatted as FAT32"));
    Serial.println(F("  - Card is properly inserted"));
    Serial.println(F("  - SPI wiring is correct"));
    g_sdAvailable = false;

    // Post alert via network
    extern void network_sendAlert(const char* title, const char* message);
    network_sendAlert("SD Card Error", "MicroSD card failed to mount. Logging and persistence disabled.");
    return false;
  }

  // Check card type and size
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println(F("[SD] No SD card attached"));
    g_sdAvailable = false;
    return false;
  }

  Serial.print(F("[SD] Card Type: "));
  if (cardType == CARD_MMC) {
    Serial.println(F("MMC"));
  } else if (cardType == CARD_SD) {
    Serial.println(F("SDSC"));
  } else if (cardType == CARD_SDHC) {
    Serial.println(F("SDHC"));
  } else {
    Serial.println(F("UNKNOWN"));
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.print(F("[SD] Card Size: "));
  Serial.print((uint32_t)cardSize);
  Serial.println(F(" MB"));

  // Create log directory if it doesn't exist
  if (!SD.exists("/logs")) {
    SD.mkdir("/logs");
    Serial.println(F("[SD] Created /logs directory"));
  }

  // Create profiles directory for adaptive learning
  if (!SD.exists("/profiles")) {
    SD.mkdir("/profiles");
    Serial.println(F("[SD] Created /profiles directory"));
  }

  // Initialize today's log file
  g_currentLogFile = buildDateFilename();
  if (!fileExists(g_currentLogFile.c_str())) {
    sdLogger_writeHeader(g_currentLogFile.c_str());
  }

  // Check existing files for corruption
  sdLogger_checkFileIntegrity(g_currentLogFile.c_str());
  sdLogger_checkFileIntegrity(SUMMARY_ARCHIVE_FILE);

  // GH-SYS-001: Load cached runtime parameters
  sdLogger_loadCache();

  // GH-SAFE-002 persistent: Restore compressor cooldown state
  sdLogger_loadCooldownState();

  // Purge logs older than 30 days
  sdLogger_purgeOldLogs();

  g_sdAvailable = true;
  Serial.println(F("[SD] MicroSD initialized successfully"));
  return true;
}

bool sdLogger_isAvailable() {
  return g_sdAvailable;
}

bool sdLogger_writeHeader(const char* filename) {
  String header = "Timestamp,Temperature(C),Humidity(%),CO2(ppm),FridgeTemp(C),"
                  "HOH,AirAssist,ExhaustFan,Compressor,NightMode";

  File f = SD.open(filename, FILE_WRITE);
  if (!f) {
    Serial.print(F("[SD] Failed to create log file: "));
    Serial.println(filename);
    return false;
  }

  f.println(header);
  f.close();

  Serial.print(F("[SD] Created new log file: "));
  Serial.println(filename);
  return true;
}

bool sdLogger_writeData() {
  // GH-LOG-001: Log ambient data every 60 seconds
  if (!g_sdAvailable) return false;

  // Check if date has changed (new day = new file)
  String todayFile = buildDateFilename();
  if (todayFile != g_currentLogFile) {
    g_currentLogFile = todayFile;
    if (!fileExists(g_currentLogFile.c_str())) {
      sdLogger_writeHeader(g_currentLogFile.c_str());
    }
    // Purge old logs when crossing midnight
    sdLogger_purgeOldLogs();
  }

  // Build log entry
  LogEntry entry;
  strncpy(entry.timestamp, rtc_getTimeString(), sizeof(entry.timestamp) - 1);
  entry.timestamp[sizeof(entry.timestamp) - 1] = '\0';

  entry.temperature = sensors_isTemperatureValid() ?
                      g_systemState.currentTemp :
                      g_systemState.lastKnownGoodTemp;
  entry.humidity = sensors_isHumidityValid() ?
                   g_systemState.currentHumidity :
                   g_systemState.lastKnownGoodHumidity;
  entry.co2 = sensors_isCO2Valid() ?
              g_systemState.currentCO2 :
              g_systemState.lastKnownGoodCO2;
  entry.fridgeTemp = network_getFridgeTemp();
  entry.hoHActive = g_systemState.hoHActive;
  entry.airAssistActive = g_systemState.airAssistActive;
  entry.exhaustFanActive = g_systemState.exhaustFanActive;
  entry.compressorActive = g_systemState.compressorActive;
  entry.nightMode = g_systemState.nightModeActive;

  // Format CSV line
  String line = String(entry.timestamp) + ",";
  line += String(entry.temperature, 2) + ",";
  line += String(entry.humidity, 2) + ",";
  line += String(entry.co2) + ",";
  line += String(entry.fridgeTemp, 2) + ",";
  line += (entry.hoHActive ? "1" : "0") + String(",");
  line += (entry.airAssistActive ? "1" : "0") + String(",");
  line += (entry.exhaustFanActive ? "1" : "0") + String(",");
  line += (entry.compressorActive ? "1" : "0") + String(",");
  line += (entry.nightMode ? "1" : "0");

  bool writeSuccess = writeLine(g_currentLogFile.c_str(), line);

  // GH-SAFE-002 persistent: Save cooldown state on every log write
  // Also save when compressor state changes (detected by edge)
  bool compressorCurrentlyOn = g_systemState.compressorActive;
  if (compressorCurrentlyOn != g_lastCompressorState) {
    g_lastCompressorState = compressorCurrentlyOn;

    if (!compressorCurrentlyOn && relayManager_isCompressorCooldownActive()) {
      // Compressor just turned off and cooldown is active - persist immediately
      sdLogger_saveCooldownState();
      Serial.println(F("[SD] Compressor OFF - cooldown state persisted"));
    }
  }

  // Periodic cooldown persistence even without state changes
  static unsigned long lastCooldownSave = 0;
  if (millis() - lastCooldownSave >= 300000UL) {  // Every 5 minutes
    lastCooldownSave = millis();
    if (relayManager_isCompressorCooldownActive()) {
      sdLogger_saveCooldownState();
    }
  }

  return writeSuccess;
}

// ============================================
// Runtime Cache (GH-SYS-001)
// ============================================

bool sdLogger_loadCache() {
  if (!g_sdAvailable) return false;

  const char* cacheFile = "/cache.dat";

  if (!SD.exists(cacheFile)) {
    Serial.println(F("[SD] No cache file found - using factory defaults"));
    // Initialize cache with safe defaults
    g_runtimeCache.totalRuntimeHours = 0;
    g_runtimeCache.lastActiveBand = 0;
    g_runtimeCache.emaWeight = DEFAULT_EMA_WEIGHT;
    g_runtimeCache.compressorLastOffTimestamp = 0;
    g_runtimeCache.compressorCooldownActive = false;
    return false;
  }

  File f = SD.open(cacheFile, FILE_READ);
  if (!f) {
    Serial.println(F("[SD] Cannot open cache file - using defaults"));
    return false;
  }

  size_t bytesRead = f.read((uint8_t*)&g_runtimeCache, sizeof(RuntimeCache));
  f.close();

  if (bytesRead == sizeof(RuntimeCache)) {
    // Validate loaded data ranges
    bool cacheValid = true;

    if (g_runtimeCache.emaWeight < EMA_WEIGHT_MIN || g_runtimeCache.emaWeight > EMA_WEIGHT_MAX) {
      Serial.println(F("[SD] Cache: EMA weight out of range - resetting"));
      g_runtimeCache.emaWeight = DEFAULT_EMA_WEIGHT;
      cacheValid = false;
    }

    if (g_runtimeCache.totalRuntimeHours > 1000000) {
      Serial.println(F("[SD] Cache: Runtime hours implausible - resetting"));
      g_runtimeCache.totalRuntimeHours = 0;
      cacheValid = false;
    }

    if (g_runtimeCache.lastActiveBand > 3) {
      Serial.println(F("[SD] Cache: Invalid band index - resetting"));
      g_runtimeCache.lastActiveBand = 0;
      cacheValid = false;
    }

    if (cacheValid) {
      Serial.println(F("[SD] Loaded runtime cache successfully"));
      Serial.print(F("[SD]   Runtime: "));
      Serial.print(g_runtimeCache.totalRuntimeHours);
      Serial.println(F(" hours"));
      Serial.print(F("[SD]   EMA weight: "));
      Serial.println(g_runtimeCache.emaWeight, 2);
      Serial.print(F("[SD]   Last band: "));
      Serial.println(g_runtimeCache.lastActiveBand);
      Serial.print(F("[SD]   Cooldown was active: "));
      Serial.println(g_runtimeCache.compressorCooldownActive ? "YES" : "no");
      return true;
    }
  } else {
    Serial.println(F("[SD] Cache file size mismatch - may be corrupted"));
  }

  return false;
}

bool sdLogger_saveCache() {
  if (!g_sdAvailable) return false;

  const char* cacheFile = "/cache.dat";

  File f = SD.open(cacheFile, FILE_WRITE);
  if (!f) {
    Serial.println(F("[SD] Failed to save runtime cache"));
    return false;
  }

  size_t written = f.write((uint8_t*)&g_runtimeCache, sizeof(RuntimeCache));
  f.close();

  if (written == sizeof(RuntimeCache)) {
    return true;
  }

  Serial.println(F("[SD] Cache write incomplete - card may be full"));
  return false;
}

// ============================================
// Compressor Cooldown Persistence (GH-SAFE-002 persistent)
// ============================================

bool sdLogger_saveCooldownState() {
  if (!g_sdAvailable) return false;

  // Update cache with current cooldown state
  g_runtimeCache.compressorCooldownActive = relayManager_isCompressorCooldownActive();

  // Store RTC timestamp for accurate cross-reboot time comparison
  g_runtimeCache.compressorLastOffTimestamp = rtc_getEpochSeconds();

  // Save entire cache (includes all runtime params plus cooldown state)
  bool saved = sdLogger_saveCache();

  if (saved) {
    Serial.print(F("[SD] Cooldown state persisted: "));
    Serial.print(g_runtimeCache.compressorCooldownActive ? "ACTIVE" : "inactive");
    if (g_runtimeCache.compressorLastOffTimestamp > 0) {
      Serial.print(F(", timestamp: "));
      Serial.print(g_runtimeCache.compressorLastOffTimestamp);
    }
    Serial.println();
  }

  return saved;
}

bool sdLogger_loadCooldownState() {
  if (!g_sdAvailable) {
    Serial.println(F("[SD] No SD - cooldown state not restored"));
    return false;
  }

  if (g_runtimeCache.compressorCooldownActive) {
    Serial.println(F("[SD] Restoring compressor cooldown state from cache..."));

    // Calculate elapsed time since last compressor OFF
    unsigned long elapsedSinceOff = 0;
    unsigned long currentTimestamp = rtc_getEpochSeconds();

    if (currentTimestamp > 0 && g_runtimeCache.compressorLastOffTimestamp > 0) {
      if (currentTimestamp > g_runtimeCache.compressorLastOffTimestamp) {
        elapsedSinceOff = currentTimestamp - g_runtimeCache.compressorLastOffTimestamp;
      }
    }

    Serial.print(F("[SD] Time since compressor OFF: "));
    Serial.print(elapsedSinceOff);
    Serial.println(F(" seconds"));

    // Pass to relay manager to restore the cooldown timer
    extern void relayManager_loadCooldownState(unsigned long lastOffTimestamp, bool wasInCooldown);
    relayManager_loadCooldownState(
      g_runtimeCache.compressorLastOffTimestamp,
      g_runtimeCache.compressorCooldownActive
    );

    // If cooldown has expired during downtime, update the cache
    if (elapsedSinceOff >= COMPRESSOR_COOLDOWN_SEC) {
      g_runtimeCache.compressorCooldownActive = false;
      sdLogger_saveCache();
      Serial.println(F("[SD] Cooldown expired during downtime - cache updated"));
    }

    return true;
  }

  Serial.println(F("[SD] No active cooldown to restore"));
  return false;
}

// ============================================
// Log Management
// ============================================

bool sdLogger_purgeOldLogs() {
  // GH-LOG-002: Remove log files older than 30 days
  if (!g_sdAvailable) return false;

  RTCTime now;
  if (!rtc_readTime(&now)) {
    Serial.println(F("[SD] Cannot purge logs - RTC unavailable"));
    return false;
  }

  File root = SD.open("/logs");
  if (!root) {
    Serial.println(F("[SD] Cannot open /logs directory for purge"));
    return false;
  }

  int purgedCount = 0;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    String filename = entry.name();
    entry.close();

    // Only process log CSV files
    if (!filename.startsWith("log_") || !filename.endsWith(".csv")) {
      continue;
    }

    // Extract date from filename: log_YYYY-MM-DD.csv
    if (filename.length() < 15) continue;

    int fileYear = filename.substring(4, 8).toInt();
    int fileMonth = filename.substring(9, 11).toInt();
    int fileDay = filename.substring(12, 14).toInt();

    if (fileYear == 0 || fileMonth == 0 || fileDay == 0) continue;

    // Calculate approximate age in days
    int fileTotalDays = fileYear * 365 + fileMonth * 30 + fileDay;
    int nowTotalDays = now.year * 365 + now.month * 30 + now.date;
    int ageDays = nowTotalDays - fileTotalDays;

    if (ageDays > LOG_RETENTION_DAYS) {
      String fullPath = "/logs/" + filename;
      Serial.print(F("[SD] Purging old log: "));
      Serial.print(fullPath);
      Serial.print(F(" ("));
      Serial.print(ageDays);
      Serial.println(F(" days old)"));

      if (SD.remove(fullPath)) {
        purgedCount++;
      } else {
        Serial.print(F("[SD] Failed to purge: "));
        Serial.println(fullPath);
      }
    }
  }

  root.close();

  if (purgedCount > 0) {
    Serial.print(F("[SD] Purged "));
    Serial.print(purgedCount);
    Serial.println(F(" old log files"));
  }

  return true;
}

bool sdLogger_checkFileIntegrity(const char* path) {
  // GH-LOG-003: Check if file is readable
  if (!g_sdAvailable) return false;

  if (!SD.exists(path)) {
    // File doesn't exist - that's OK, not corruption
    return true;
  }

  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.print(F("[SD] CORRUPTION DETECTED: Cannot open "));
    Serial.println(path);

    // Try to remove corrupted file
    if (SD.remove(path)) {
      Serial.print(F("[SD] Removed corrupted file: "));
      Serial.println(path);

      extern void network_sendAlert(const char* title, const char* message);
      network_sendAlert("SD Card Corruption",
                       ("Corrupted file detected and removed: " + String(path)).c_str());
    }

    return false;
  }

  // File is readable - check it has reasonable size
  size_t fileSize = f.size();
  f.close();

  if (fileSize == 0 && strstr(path, ".json") != NULL) {
    // Empty JSON profile file - flag as corrupted
    Serial.print(F("[SD] WARNING: Empty profile file: "));
    Serial.println(path);
    return false;
  }

  return true;
}

void sdLogger_logSystemEvent(const char* event) {
  if (!g_sdAvailable) return;

  const char* eventFile = "/system_events.log";

  String line = String(rtc_getTimeString()) + " | " + String(event);

  writeLine(eventFile, line);
  Serial.print(F("[SD] Event logged: "));
  Serial.println(event);
}
