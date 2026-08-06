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

// ============================================================
// SD Card Mutex (v1.4 — graph dashboard)
// ============================================================
SemaphoreHandle_t g_sdMutex = NULL;

bool sdLogger_initMutex() {
  g_sdMutex = xSemaphoreCreateMutex();
  if (g_sdMutex == NULL) {
    Serial.println(F("[SD] FATAL: Failed to create SD mutex"));
    return false;
  }
  Serial.println(F("[SD] SD mutex initialized"));
  return true;
}

static bool g_sdAvailable = false;
static SPIClass g_sdSPI(VSPI);
static String g_currentLogFile = "";

// Track compressor state changes to trigger cooldown saves

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

  // Purge logs older than 30 days
  sdLogger_purgeOldLogs();

  Serial.println(F("[SD] MicroSD initialized successfully"));
  return true;
}

bool sdLogger_isAvailable() {
  return g_sdAvailable;
}

String sdLogger_getCurrentLogFilename() {
  return g_currentLogFile;
}

bool sdLogger_writeHeader(const char* filename) {
    const char* header = "Timestamp,Temperature(C),Humidity(%),CO2(ppm),FridgeTemp(C),FridgeHum(%),FridgeDoor,"
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
  entry.fridgeHumidity = network_getFridgeHumidity();
  entry.fridgeDoorOpen = network_isFridgeDoorOpen();
  entry.hoHActive = g_systemState.hoHActive;
  entry.airAssistActive = g_systemState.airAssistActive;
  entry.exhaustFanActive = g_systemState.exhaustFanActive;
  entry.compressorActive = g_systemState.compressorActive;
  entry.nightMode = g_systemState.nightModeActive;

  char line[128];
    int len = snprintf(line, sizeof(line), "%s,%.2f,%.2f,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d",
                     entry.timestamp,
                     entry.temperature,
                     entry.humidity,
                     entry.co2,
                     entry.fridgeTemp,
                     entry.fridgeHumidity,
                     entry.fridgeDoorOpen ? 1 : 0,
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
    g_runtimeCache.version = 2;
    g_runtimeCache.totalRuntimeHours = 0;
    g_runtimeCache.lastActiveBand = 0;
    g_runtimeCache.emaWeight = DEFAULT_EMA_WEIGHT;
    g_runtimeCache.compressorLastOffTimestamp = 0;
    g_runtimeCache.compressorCooldownActive = false;
    const RelayMapping* defMap = relayManager_getDefaultMapping();
    memcpy(&g_runtimeCache.relayMapping, defMap, sizeof(RelayMapping));
    return false;
  }

  File f = SD.open(cacheFile, FILE_READ);
   if (!f) {
    Serial.println(F("[SD] Cannot open cache file - using defaults"));
    g_runtimeCache.version = 2;
    const RelayMapping* defMap = relayManager_getDefaultMapping();
    memcpy(&g_runtimeCache.relayMapping, defMap, sizeof(RelayMapping));
    return false;
  }

   size_t bytesRead = f.read((uint8_t*)&g_runtimeCache, sizeof(RuntimeCache));

  if (bytesRead == sizeof(RuntimeCache)) {
    bool cacheValid = true;

    if (g_runtimeCache.version == 0 || g_runtimeCache.version > 2) {
      Serial.println(F("[SD] Cache: Unknown version — resetting"));
      cacheValid = false;
    }

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
              // Restore relay mapping from cache (v1.4)
      if (g_runtimeCache.relayMapping.magic == RELAY_MAPPING_MAGIC) {
        Serial.print(F("[SD]   Relay mapping: valid"));
        Serial.println();
      } else {
        Serial.print(F("[SD]   Relay mapping: not set (using defaults)"));
        Serial.println();
      }
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

    g_runtimeCache.version = 2;
    g_runtimeCache.totalRuntimeHours = 0;
    g_runtimeCache.lastActiveBand = 0;
    g_runtimeCache.emaWeight = DEFAULT_EMA_WEIGHT;
    g_runtimeCache.compressorLastOffTimestamp = 0;
    g_runtimeCache.compressorCooldownActive = false;

    // Initialize relay mapping to defaults
    const RelayMapping* defMap = relayManager_getDefaultMapping();
    memcpy(&g_runtimeCache.relayMapping, defMap, sizeof(RelayMapping));

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

// ============================================================
// Graph Dashboard Helpers (v1.4)
// ============================================================

// Convert a Unix day number (days since 1970-01-01) to a log filename.
static void epochDayToFilename(uint32_t unixDay, char* out, size_t outLen) {
  uint32_t days = unixDay;
  int y = 1970;
  while (true) {
    bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    uint32_t daysInYear = leap ? 366UL : 365UL;
    if (days < daysInYear) break;
    days -= daysInYear;
    y++;
  }
  static const uint8_t monthDays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int m = 1;
  for (; m <= 12; m++) {
    uint8_t md = monthDays[m-1];
    if (m == 2 && (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))) md = 29;
    if (days < md) break;
    days -= md;
  }
  uint8_t d = days + 1;
  snprintf(out, outLen, "/logs/log_%04d-%02d-%02d.csv", y, m, d);
}

// Convert broken-out date/time to Unix epoch. Returns 0 if invalid.
static unsigned long dateTimeToUnixEpoch(int y, int m, int d, int h, int min, int s) {
  if (y < 1970 || y > 2099 || m < 1 || m > 12 || d < 1 || d > 31 ||
      h < 0 || h > 23 || min < 0 || min > 59 || s < 0 || s > 59) return 0;

  static const uint8_t monthDays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  uint8_t maxDay = monthDays[m-1];
  if (m == 2 && (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))) maxDay = 29;
  if (d > maxDay) return 0;

  unsigned long days = 0;
  for (int yr = 1970; yr < y; yr++) {
    days += (yr % 4 == 0 && (yr % 100 != 0 || yr % 400 == 0)) ? 366UL : 365UL;
  }
  for (int mo = 1; mo < m; mo++) {
    uint8_t md = monthDays[mo-1];
    if (mo == 2 && (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))) md = 29;
    days += md;
  }
  days += d - 1;
  return days * 86400UL + h * 3600UL + min * 60UL + s;
}

// Parse one CSV line to extract epoch and sensor value.
// targetCol: 0=temp, 1=hum, 2=CO2, 3=fridgeTemp
// Returns true if valid point found.
static bool parseGraphPoint(const char* line, size_t lineLen, int targetCol,
                            unsigned long* outEpoch, float* outValue) {
  if (lineLen < 19) return false;

  int y, m, d, h, min, sec;
  if (sscanf(line, "%4d-%2d-%2d %2d:%2d:%2d", &y, &m, &d, &h, &min, &sec) != 6) return false;

  unsigned long epoch = dateTimeToUnixEpoch(y, m, d, h, min, sec);
  if (epoch == 0) return false;
  *outEpoch = epoch;

  // Walk commas to target column (1 + targetCol = CSV field index after timestamp)
  int csvCol = 1 + targetCol;
  int currentCol = 0;
  const char* p = line;
  const char* lineEnd = line + lineLen;

  while (p < lineEnd && *p != ',' && *p != '\0') p++;
  if (p >= lineEnd || *p != ',') return false;
  p++;
  currentCol = 1;

  while (currentCol < csvCol && p < lineEnd) {
    while (p < lineEnd && *p != ',' && *p != '\0') p++;
    if (p >= lineEnd) return false;
    p++;
    currentCol++;
  }

  if (currentCol != csvCol) return false;

  const char* fieldStart = p;
  while (p < lineEnd && *p != ',' && *p != '\r' && *p != '\n' && *p != '\0') p++;
  size_t fieldLen = p - fieldStart;

  if (fieldLen == 0) return false;

  char fieldBuffer[32];
  if (fieldLen >= sizeof(fieldBuffer)) fieldLen = sizeof(fieldBuffer) - 1;
  memcpy(fieldBuffer, fieldStart, fieldLen);
  fieldBuffer[fieldLen] = '\0';

  char* endPtr;
  *outValue = strtof(fieldBuffer, &endPtr);
  if (endPtr == fieldBuffer) return false;
  if (isnan(*outValue) || isinf(*outValue)) return false;

  return true;
}

// ============================================================
// Graph Dashboard — Historical Data Query (v1.4)
// ============================================================
// Two-pass algorithm:
//   Pass 1: Count valid points to calculate downsampling stride.
//   Pass 2: Read files newest-to-oldest, emit every Nth point.
//   Mutex taken per-file with 200ms timeout.
//   Time-based yield every GRAPH_YIELD_INTERVAL_MS.
//   Returns 350 points max (proven safe for 8KB buffer).

size_t sdLogger_getHistoricalData(const GraphDataRequest* request, char* output, size_t outputMax) {
  if (!g_sdAvailable || !request || !output || outputMax < 256) return 0;
  if (request->sensorType > 3) return 0;

  uint32_t startEpoch = request->startEpoch;
  uint32_t endEpoch = request->endEpoch;
  uint16_t maxPoints = request->maxPoints;
  uint32_t requestId = request->requestId;
  uint8_t sensorType = request->sensorType;

  if (maxPoints == 0 || maxPoints > GRAPH_MAX_RESPONSE_POINTS) {
    maxPoints = GRAPH_MAX_RESPONSE_POINTS;
  }

  RTCTime now;
  if (!rtc_readTime(&now)) {
    return snprintf(output, outputMax,
                    "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[]}",
                    sensorType, requestId);
  }
  unsigned long gh2000 = rtc_timeToGH2000Seconds(&now);
  unsigned long unixNow = gh2000 + 946684800UL;

  if (endEpoch == 0) endEpoch = unixNow;
  if (startEpoch >= endEpoch) {
    return snprintf(output, outputMax,
                    "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[]}",
                    sensorType, requestId);
  }
  if (endEpoch > unixNow) endEpoch = unixNow;

  uint32_t startDay = startEpoch / 86400UL;
  uint32_t endDay = endEpoch / 86400UL;
  uint32_t todayDay = unixNow / 86400UL;

  // ================================================================
  // PASS 1: Count valid points
  // ================================================================
  uint32_t totalPoints = 0;
  char filename[40];
  char lineBuffer[512];
  unsigned long lastYield = millis();

  for (uint32_t day = startDay; day <= endDay && day <= todayDay; day++) {
    epochDayToFilename(day, filename, sizeof(filename));
    if (!SD.exists(filename)) continue;

    if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
      return snprintf(output, outputMax,
                      "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[]}",
                      sensorType, requestId);
    }

    File f = SD.open(filename, FILE_READ);
    if (f) {
      char header[256];
      f.readBytesUntil('\n', header, sizeof(header));

      while (f.available()) {
        size_t bytesRead = f.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
        lineBuffer[bytesRead] = '\0';

        if (bytesRead == sizeof(lineBuffer) - 1 && lineBuffer[bytesRead-1] != '\n') {
          while (f.available() && f.read() != '\n');
          continue;
        }
        if (bytesRead < 19) continue;

        unsigned long pointEpoch;
        float pointValue;
        if (parseGraphPoint(lineBuffer, bytesRead, sensorType, &pointEpoch, &pointValue)) {
          if (pointEpoch >= startEpoch && pointEpoch <= endEpoch) {
            totalPoints++;
          }
        }

        if (millis() - lastYield >= GRAPH_YIELD_INTERVAL_MS) {
          yield();
          lastYield = millis();
        }
      }
      f.close();
    }
    xSemaphoreGive(g_sdMutex);
  }

  // ================================================================
  // Build response
  // ================================================================
  if (totalPoints == 0) {
    return snprintf(output, outputMax,
                    "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[]}",
                    sensorType, requestId);
  }

  uint32_t stride = 1;
  if (totalPoints > maxPoints) {
    stride = totalPoints / maxPoints;
    if (stride == 0) stride = 1;
  }

  size_t pos = 0;
  pos += snprintf(output + pos, outputMax - pos,
                  "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[",
                  sensorType, requestId);

  // ================================================================
  // PASS 2: Emit uniformly-downsampled points (newest files first)
  // ================================================================
  uint32_t counted = 0;
  uint32_t emitted = 0;
  lastYield = millis();

  for (uint32_t day = endDay; day >= startDay; day--) {
    if (emitted >= maxPoints) break;

    epochDayToFilename(day, filename, sizeof(filename));
    if (!SD.exists(filename)) continue;

    if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
      pos = snprintf(output, outputMax,
                     "{\"type\":100,\"error\":\"SD_TIMEOUT\",\"rid\":%u}",
                     requestId);
      return pos;
    }

    File f = SD.open(filename, FILE_READ);
    if (f) {
      char header[256];
      f.readBytesUntil('\n', header, sizeof(header));

      while (f.available() && emitted < maxPoints) {
        if (pos + 64 > outputMax) {
          f.close();
          xSemaphoreGive(g_sdMutex);
          pos = snprintf(output, outputMax,
                         "{\"type\":100,\"error\":\"BUFFER_FULL\",\"rid\":%u}",
                         requestId);
          return pos;
        }

        size_t bytesRead = f.readBytesUntil('\n', lineBuffer, sizeof(lineBuffer) - 1);
        lineBuffer[bytesRead] = '\0';

        if (bytesRead == sizeof(lineBuffer) - 1 && lineBuffer[bytesRead-1] != '\n') {
          while (f.available() && f.read() != '\n');
          continue;
        }
        if (bytesRead < 19) continue;

        unsigned long pointEpoch;
        float pointValue;
        if (!parseGraphPoint(lineBuffer, bytesRead, sensorType, &pointEpoch, &pointValue)) continue;
        if (pointEpoch < startEpoch || pointEpoch > endEpoch) continue;

        if (counted % stride == 0) {
          if (emitted > 0) {
            pos += snprintf(output + pos, outputMax - pos, ",");
          }
          pos += snprintf(output + pos, outputMax - pos, "[%lu,%.2f]", pointEpoch, pointValue);
          emitted++;
        }
        counted++;

        if (millis() - lastYield >= GRAPH_YIELD_INTERVAL_MS) {
          yield();
          lastYield = millis();
        }
      }
      f.close();
    }
    xSemaphoreGive(g_sdMutex);
  }

  pos += snprintf(output + pos, outputMax - pos, "]}");
  return pos;
}
