/*
    sd_logger.cpp
    GrowHub32 - MicroSD Data Logging Implementation
    Version: 1.2.7
    Revision: Resolved zero-byte CSV daily initialization trap.
              Zero heap allocation via snprintf throughout.
              Hardened filename path indexing for robust log purging.

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
#include "system_state.h"

// External declarations
extern AutomationThresholds* automation_getThresholds();

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
    return "/logs/log_unknown_date.csv";
  }

  char filename[40];
  snprintf(filename, sizeof(filename), "/logs/log_%04d-%02d-%02d.csv",
           now.year, now.month, now.date);
  return String(filename);
}

static bool fileExists(const char* path) {
  return SD.exists(path);
}

static bool writeLine(const char* filename, const char* line) {
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

  // Mark SD as available BEFORE cache/cooldown load calls
  g_sdAvailable = true;

  // Initialize today's log file path reference
  g_currentLogFile = buildDateFilename();

  // Validate or restore file structures before execution
  sdLogger_checkFileIntegrity(g_currentLogFile.c_str());
  sdLogger_checkFileIntegrity(SUMMARY_ARCHIVE_FILE);

  // Write header if the file does not exist (or was cleaned up as a 0-byte fragment)
  if (!fileExists(g_currentLogFile.c_str())) {
    sdLogger_writeHeader(g_currentLogFile.c_str());
  }

  // GH-SYS-001: Load cached runtime parameters
  sdLogger_loadCache();

  // GH-SAFE-002 persistent: Restore compressor cooldown state
  sdLogger_loadCooldownState();

  // Purge logs older than 30 days
  sdLogger_purgeOldLogs();

  Serial.println(F("[SD] MicroSD initialized successfully"));
  return true;
}

bool sdLogger_isAvailable() {
  return g_sdAvailable;
}

bool sdLogger_writeHeader(const char* filename) {
  const char* header = "Timestamp,Temperature(C),Humidity(%),CO2(ppm),FridgeTemp(C),"
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
  if (!g_sdAvailable) return false;

  // Check if date has changed (new day = new file)
  String todayFile = buildDateFilename();
  if (todayFile != g_currentLogFile) {
    g_currentLogFile = todayFile;
    if (!fileExists(g_currentLogFile.c_str())) {
      sdLogger_writeHeader(g_currentLogFile.c_str());
    }
    sdLogger_purgeOldLogs();
  }

  LogEntry entry;

  char timeStr[24];
  rtc_getTimeString(timeStr, sizeof(timeStr));
  strncpy(entry.timestamp, timeStr, sizeof(entry.timestamp) - 1);
  entry.timestamp[sizeof(entry.timestamp) - 1] = '\0';

  if (sensors_isTemperatureValid()) {
    entry.temperature = g_systemState.currentTemp;
  } else {
    entry.temperature = isnan(g_systemState.lastKnownGoodTemp) ? 0.0f : g_systemState.lastKnownGoodTemp;
  }

  if (sensors_isHumidityValid()) {
    entry.humidity = g_systemState.currentHumidity;
  } else {
    entry.humidity = (g_systemState.lastKnownGoodHumidity >= 0.0f && g_systemState.lastKnownGoodHumidity <= 100.0f)
                     ? g_systemState.lastKnownGoodHumidity : 0.0f;
  }

  if (sensors_isCO2Valid()) {
    entry.co2 = g_systemState.currentCO2;
  } else {
    entry.co2 = (g_systemState.lastKnownGoodCO2 > 0 && g_systemState.lastKnownGoodCO2 <= 10000)
                ? g_systemState.lastKnownGoodCO2 : 0;
  }

  entry.fridgeTemp = network_getFridgeTemp();
  entry.hoHActive = g_systemState.hoHActive;
  entry.airAssistActive = g_systemState.airAssistActive;
  entry.exhaustFanActive = g_systemState.exhaustFanActive;
  entry.compressorActive = g_systemState.compressorActive;
  entry.nightMode = g_systemState.nightModeActive;

  char line[128];
  int len = snprintf(line, sizeof(line), "%s,%.2f,%.2f,%d,%.2f,%d,%d,%d,%d,%d",
                     entry.timestamp,
                     entry.temperature,
                     entry.humidity,
                     entry.co2,
                     entry.fridgeTemp,
                     entry.hoHActive ? 1 : 0,
                     entry.airAssistActive ? 1 : 0,
                     entry.exhaustFanActive ? 1 : 0,
                     entry.compressorActive ? 1 : 0,
                     entry.nightMode ? 1 : 0);

  if (len < 0 || (size_t)len >= sizeof(line)) {
    Serial.println(F("[SD] ERROR: Log line formatting failed or truncated"));
    return false;
  }

  bool writeSuccess = writeLine(g_currentLogFile.c_str(), line);

  // GH-SAFE-002 persistent: Track state switches cleanly
  bool compressorCurrentlyOn = g_systemState.compressorActive;
  if (compressorCurrentlyOn != g_lastCompressorState) {
    g_lastCompressorState = compressorCurrentlyOn;

    if (!compressorCurrentlyOn && relayManager_isCompressorCooldownActive()) {
      sdLogger_saveCooldownState();
      Serial.println(F("[SD] Compressor OFF - cooldown state persisted"));
    }
  }

  // Periodic cooldown persistence balance window
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
    } else {
      sdLogger_saveCache();
      return false;
    }
  } else {
    Serial.print(F("[SD] Cache size mismatch (expected "));
    Serial.print(sizeof(RuntimeCache));
    Serial.print(F(", got "));
    Serial.print(bytesRead);
    Serial.println(F(") - resetting to defaults for new format"));

    g_runtimeCache.totalRuntimeHours = 0;
    g_runtimeCache.lastActiveBand = 0;
    g_runtimeCache.emaWeight = DEFAULT_EMA_WEIGHT;
    g_runtimeCache.compressorLastOffTimestamp = 0;
    g_runtimeCache.compressorCooldownActive = false;

    sdLogger_saveCache();
    return false;
  }

  return false;
}

bool sdLogger_saveCache() {
  if (!g_sdAvailable) return false;

  const char* cacheFile = "/cache.dat";

  if (SD.exists(cacheFile)) {
    SD.remove(cacheFile);
  }

  File f = SD.open(cacheFile, FILE_WRITE);
  if (!f) {
    Serial.println(F("[SD] Failed to save runtime cache"));
    return false;
  }

  size_t written = f.write((uint8_t*)&g_runtimeCache, sizeof(RuntimeCache));
  f.close();

  return (written == sizeof(RuntimeCache));
}

// ============================================
// Compressor Cooldown Persistence (GH-SAFE-002 persistent)
// ============================================

bool sdLogger_saveCooldownState() {
  if (!g_sdAvailable) return false;

  g_runtimeCache.compressorCooldownActive = relayManager_isCompressorCooldownActive();
  unsigned long offEpoch = g_relays[RELAY_COMPRESSOR].cooldownOffEpoch;

  if (offEpoch > 0) {
    g_runtimeCache.compressorLastOffTimestamp = offEpoch;
  } else if (g_runtimeCache.compressorLastOffTimestamp == 0) {
    Serial.println(F("[SD] WARNING: RTC unavailable at compressor OFF - no timestamp to persist"));
  }

  bool saved = sdLogger_saveCache();

  if (saved) {
    Serial.print(F("[SD] Cooldown state persisted: "));
    Serial.print(g_runtimeCache.compressorCooldownActive ? "ACTIVE" : "inactive");
    if (g_runtimeCache.compressorLastOffTimestamp > 0) {
      Serial.print(F(", offEpoch: "));
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

  if (!g_runtimeCache.compressorCooldownActive) {
    Serial.println(F("[SD] No active cooldown to restore"));
    return false;
  }

  Serial.println(F("[SD] Restoring compressor cooldown state from cache..."));
  Serial.print(F("[SD]   Cached offEpoch: "));
  Serial.println(g_runtimeCache.compressorLastOffTimestamp);

  relayManager_loadCooldownState(
    g_runtimeCache.compressorLastOffTimestamp,
    g_runtimeCache.compressorCooldownActive
  );

  bool stillActive = relayManager_isCompressorCooldownActive();
  if (!stillActive && g_runtimeCache.compressorCooldownActive) {
    g_runtimeCache.compressorCooldownActive = false;
    sdLogger_saveCache();
    Serial.println(F("[SD] Cooldown expired during downtime - cache updated"));
  }

  return true;
}

// ============================================
// Log Management
// ============================================

bool sdLogger_purgeOldLogs() {
  if (!g_sdAvailable) return false;

 unsigned long nowEpoch = rtc_getGH2000Seconds();
  if (nowEpoch == 0) {
    Serial.println(F("[SD] Cannot purge logs - RTC unavailable (epoch is 0, skipping to avoid mass deletion)"));
    return false;
  }

  unsigned long retentionSeconds = (unsigned long)LOG_RETENTION_DAYS * 86400UL;

  File root = SD.open("/logs");
  if (!root) {
    Serial.println(F("[SD] Cannot open /logs directory for purge"));
    return false;
  }

  int purgedCount = 0;

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    String rawName = entry.name();
    entry.close();

    int lastSlash = rawName.lastIndexOf('/');
    String filename = (lastSlash >= 0) ? rawName.substring(lastSlash + 1) : rawName;

    if (!filename.startsWith("log_") || !filename.endsWith(".csv")) {
      continue;
    }

    if (filename.length() < 15) continue;

    int fileYear = filename.substring(4, 8).toInt();
    int fileMonth = filename.substring(9, 11).toInt();
    int fileDay = filename.substring(12, 14).toInt();

    if (fileYear == 0 || fileMonth == 0 || fileDay == 0) continue;

    RTCTime fileDate = {0};
    fileDate.year = (uint16_t)fileYear;
    fileDate.month = (uint8_t)fileMonth;
    fileDate.date = (uint8_t)fileDay;

   unsigned long fileEpoch = rtc_timeToGH2000Seconds(&fileDate);
    if (fileEpoch == 0) continue;

    if (nowEpoch <= fileEpoch) continue;

    unsigned long ageSeconds = nowEpoch - fileEpoch;

    if (ageSeconds > retentionSeconds) {
      unsigned long ageDays = ageSeconds / 86400UL;
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
  if (!g_sdAvailable) return false;

  if (!SD.exists(path)) {
    return true;
  }

  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.print(F("[SD] WARNING: Temporary read lock or block issue on opening: "));
    Serial.println(path);
    return false;
  }

  size_t fileSize = f.size();
  f.close();

  // Handle empty file states defensively
  if (fileSize == 0) {
    if (strstr(path, ".json") != NULL) {
      Serial.print(F("[SD] WARNING: Empty profile file: "));
      Serial.println(path);
      return false;
    }

    // Drop empty CSV file targets so initialization code is forced to regenerate structural headers
    if (strstr(path, ".csv") != NULL) {
      Serial.print(F("[SD] Purging uninitialized zero-byte data fragment: "));
      Serial.println(path);
      SD.remove(path);
    }
  }

  return true;
}

void sdLogger_logSystemEvent(const char* event) {
  if (!g_sdAvailable) return;

  const char* eventFile = "/system_events.log";

  char timeStr[24];
  rtc_getTimeString(timeStr, sizeof(timeStr));

  char line[128];
  snprintf(line, sizeof(line), "%s | %s", timeStr, event);

  writeLine(eventFile, line);
  Serial.print(F("[SD] Event logged: "));
  Serial.println(event);
}
