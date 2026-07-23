/*
   rtc_handler.cpp
   GrowHub32 - DS3231 RTC Driver Implementation
   Version: 1.3.0
   Revision: Added rtc_minutesUntilNightMode() for calibration scheduling.
             Changed rtc_getTimeString() to caller-provided buffer for reentrancy.
             Fixed rtc_setTime() and rtc_readTime() to validate dates against actual
             month lengths using daysInMonth().
             Rewrote rtc_checkSerialCommand() with zero-allocation strtol parser,
             overflow protection, and correct Zeller's Congruence weekday computation.
             Added century bit documentation comment.
             Removed redundant g_currentTime.valid assignment in rtc_init().
             Fixed missing time->date assignment in rtc_readTime() (post-review fix).
             Fixed rtc_init() return value to reflect actual read success.
             Fixed year range consistency (2000-2099 throughout).
             Night mode cache rewritten as file-scope statics with g_cacheValid flag.
             Cache invalidation now correctly forces fresh RTC read after rtc_setTime().
             Fixed rtc_timeToEpoch() 32-bit overflow risk — uses uint64_t intermediate.
             Fixed I2C storm on RTC failure — retries every 60s instead of every call.
             Fixed goto crossing initialization — replaced with bool parseOk flag.
             Renamed rtc_getEpochSeconds() → rtc_getGH2000Seconds() (seconds since
             2000-01-01, NOT Unix epoch — GrowHub internal reference only).
             Renamed rtc_timeToEpoch() → rtc_timeToGH2000Seconds().

   DS3231 communicates over I2C at address 0x68.
   All time values are stored in BCD format internally.
   No DST or timezone adjustments are applied (per GH-NM-001).
*/

#include "rtc_handler.h"
#include "config.h"

static RTCTime g_currentTime;
static bool g_rtcInitialized = false;

// --- Night Mode Cache (file-scope — shared by all functions) ---
static bool   g_nightModeCached     = false;
static bool   g_nightModeCacheValid = false;
static unsigned long g_nightModeLastCheck = 0;
static bool   g_nightModeWarned     = false;

// --- Forward Declarations ---
static void rtc_invalidateNightModeCache();

// --- Private Helpers ---

static uint8_t decToBcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcdToDec(uint8_t val) {
  return ((val >> 4) * 10) + (val & 0x0F);
}

static bool isLeapYear(uint16_t year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

static uint8_t daysInMonth(uint8_t month, uint16_t year) {
  static const uint8_t days[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 0;
  if (month == 2 && isLeapYear(year)) return 29;
  return days[month];
}

static uint8_t readRegister8(uint8_t reg) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return 0;
  }

  Wire.requestFrom((uint8_t)DS3231_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

static bool writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

static bool readBurstRegisters(uint8_t startReg, uint8_t* buffer, uint8_t count) {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(startReg);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.requestFrom((uint8_t)DS3231_ADDRESS, count);
  if (Wire.available() != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

// Compute day of week from calendar date using Zeller's Congruence.
// Gregorian calendar only. Valid for GrowHub32 supported years (2000–2099).
// Returns 1=Sunday through 7=Saturday (matching DS3231 encoding).
static uint8_t computeDayOfWeek(uint16_t year, uint8_t month, uint8_t date) {
  if (month < 3) {
    month += 12;
    year -= 1;
  }
  uint8_t k = date;
  uint8_t m = month;
  uint16_t d = year % 100;
  uint16_t c = year / 100;

  // Add 700 to guarantee the numerator is never negative before applying % 7
  int h = (k + (13 * (m + 1)) / 5 + d + (d / 4) + (c / 4) - (2 * c) + 700) % 7;

  // Zeller's: 0=Saturday, 1=Sunday, 2=Monday, ..., 6=Friday
  // DS3231:   1=Sunday, 2=Monday, ..., 7=Saturday
  return (uint8_t)(h == 0 ? 7 : h);
}

// --- Public API ---

bool rtc_init() {
  Serial.println(F("[RTC] Initializing DS3231..."));

  // Check if device is present
  Wire.beginTransmission(DS3231_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[RTC] DS3231 not found on I2C bus."));
    g_rtcInitialized = false;
    return false;
  }

  // Read control register to check oscillator status
  uint8_t control = readRegister8(DS3231_REG_CONTROL);
  Serial.print(F("[RTC] Control register: 0x"));
  Serial.println(control, HEX);

  // Read status register
  uint8_t status = readRegister8(DS3231_REG_STATUS);
  Serial.print(F("[RTC] Status register: 0x"));
  Serial.println(status, HEX);

  // Check oscillator stop flag (bit 7)
  if (status & 0x80) {
    Serial.println(F("[RTC] WARNING: Oscillator was stopped. RTC may have lost time."));
    // Clear oscillator stop flag
    writeRegister8(DS3231_REG_STATUS, status & 0x7F);
  }

  // Initial time read
  bool readOk = rtc_readTime(&g_currentTime);

  if (readOk) {
    Serial.print(F("[RTC] Current time: "));
    Serial.print(g_currentTime.hours);
    Serial.print(F(":"));
    if (g_currentTime.minutes < 10) Serial.print('0');
    Serial.print(g_currentTime.minutes);
    Serial.print(F(":"));
    if (g_currentTime.seconds < 10) Serial.print('0');
    Serial.print(g_currentTime.seconds);
    Serial.print(F("  Date: "));
    Serial.print(g_currentTime.month);
    Serial.print(F("/"));
    Serial.print(g_currentTime.date);
    Serial.print(F("/"));
    Serial.println(g_currentTime.year);

    // Check if currently in night mode
    bool nightMode = rtc_isNightMode();
    Serial.print(F("[RTC] Night Mode currently: "));
    Serial.println(nightMode ? F("ACTIVE") : F("INACTIVE"));
  } else {
    Serial.println(F("[RTC] Failed to read initial time."));
  }

  g_rtcInitialized = true;
  return readOk;
}

bool rtc_readTime(RTCTime* time) {
  if (!time) return false;

  uint8_t regs[7];
  if (!readBurstRegisters(DS3231_REG_SECONDS, regs, 7)) {
    time->valid = false;
    return false;
  }

  // Mask out control bits before converting
  time->seconds   = bcdToDec(regs[0] & 0x7F);
  time->minutes   = bcdToDec(regs[1] & 0x7F);

  // Handle hours register (bit 6 = 12/24h mode, bit 5 = AM/PM in 12h mode)
  uint8_t hourReg = regs[2];
  bool is12Hour = (hourReg & 0x40) != 0;

  if (is12Hour) {
    uint8_t hour12 = bcdToDec(hourReg & 0x1F);
    bool isPM = (hourReg & 0x20) != 0;
    if (isPM && hour12 != 12) {
      time->hours = hour12 + 12;
    } else if (!isPM && hour12 == 12) {
      time->hours = 0;
    } else {
      time->hours = hour12;
    }
  } else {
    time->hours = bcdToDec(hourReg & 0x3F);
  }

  time->dayOfWeek = bcdToDec(regs[3] & 0x07);
  time->date      = bcdToDec(regs[4] & 0x3F);
  time->month     = bcdToDec(regs[5] & 0x1F);
  time->year      = 2000 + bcdToDec(regs[6]);

  // Validate ranges
  bool monthValid = (time->month >= 1 && time->month <= 12);
  uint8_t maxDay = monthValid ? daysInMonth(time->month, time->year) : 0;

  time->valid = (time->seconds < 60) &&
                (time->minutes < 60) &&
                (time->hours < 24) &&
                (time->dayOfWeek >= 1 && time->dayOfWeek <= 7) &&
                monthValid &&
                (time->date >= 1 && time->date <= maxDay) &&
                (time->year >= 2000 && time->year <= 2099);

  return time->valid;
}

bool rtc_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds,
                 uint8_t date, uint8_t month, uint16_t year,
                 uint8_t dayOfWeek) {
  if (hours > 23 || minutes > 59 || seconds > 59) return false;
  if (dayOfWeek < 1 || dayOfWeek > 7) return false;
  if (year < 2000 || year > 2099) return false;
  if (month < 1 || month > 12) return false;

  uint8_t maxDay = daysInMonth(month, year);
  if (date < 1 || date > maxDay) return false;

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_REG_SECONDS);
  Wire.write(decToBcd(seconds));
  Wire.write(decToBcd(minutes));
  Wire.write(decToBcd(hours));
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(date));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd((uint8_t)(year - 2000)));

  if (Wire.endTransmission() != 0) {
    Serial.println(F("[RTC] Failed to set time"));
    return false;
  }

  // Clear the Oscillator Stop Flag
  uint8_t status = readRegister8(DS3231_REG_STATUS);
  if (status & 0x80) {
    writeRegister8(DS3231_REG_STATUS, status & 0x7F);
    Serial.println(F("[RTC] Oscillator Stop Flag cleared after time set"));
  }

  // Invalidate night mode cache so next rtc_isNightMode() reads fresh time
  rtc_invalidateNightModeCache();

  // Refresh cached state
  rtc_readTime(&g_currentTime);

  Serial.print(F("[RTC] Time set to: "));
  Serial.print(hours); Serial.print(':');
  if (minutes < 10) Serial.print('0');
  Serial.print(minutes); Serial.print(':');
  if (seconds < 10) Serial.print('0');
  Serial.println(seconds);

  return true;
}

// ============================================================
// Night Mode Cache Invalidation
// ============================================================

static void rtc_invalidateNightModeCache() {
    g_nightModeCacheValid = false;
    g_nightModeLastCheck = 0;
}

// ============================================================
// Night Mode Check
// ============================================================

bool rtc_isNightMode() {
    // Serve cached result if valid and checked within last 60 seconds
    if (g_nightModeCacheValid && (millis() - g_nightModeLastCheck < 60000UL)) {
        return g_nightModeCached;
    }

    RTCTime now;
    if (!rtc_readTime(&now)) {
        if (!g_nightModeWarned) {
            g_nightModeWarned = true;
            Serial.println(F("[RTC] RTC unavailable - night mode disabled until recovery"));
        }
        g_nightModeCacheValid = false;
        // Keep lastCheck at current millis() so we don't retry for 60 seconds.
        // Setting to 0 would cause an I2C storm on every call to a dead RTC.
        g_nightModeLastCheck = millis();
        return false;
    }

    g_nightModeWarned = false;
    g_nightModeLastCheck = millis();
    g_nightModeCached = (now.hours >= NIGHT_MODE_START_HOUR || now.hours < NIGHT_MODE_END_HOUR);
    g_nightModeCacheValid = true;

    return g_nightModeCached;
}

float rtc_getTemperature() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_REG_TEMP_MSB);
  if (Wire.endTransmission() != 0) {
    return NAN;
  }

  Wire.requestFrom((uint8_t)DS3231_ADDRESS, (uint8_t)2);
  if (Wire.available() < 2) {
    return NAN;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  int8_t tempInt = (int8_t)msb;
  float tempFrac = (float)(lsb >> 6) * 0.25f;

  return (float)tempInt + tempFrac;
}

bool rtc_isValid() {
  if (!g_rtcInitialized) return false;

  RTCTime now;
  return rtc_readTime(&now);
}

void rtc_getTimeString(char* buffer, size_t bufferSize) {
  // "YYYY-MM-DD HH:MM:SS" = 19 chars + null = 20 bytes minimum
  if (!buffer || bufferSize < 20) {
    if (buffer && bufferSize > 0) {
      buffer[0] = '\0';
    }
    return;
  }

  RTCTime now;
  if (rtc_readTime(&now)) {
    snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d",
             now.year, now.month, now.date,
             now.hours, now.minutes, now.seconds);
  } else {
    snprintf(buffer, bufferSize, "RTC ERROR");
  }
}

// ============================================================
// v1.3: GH2000 Seconds (NOT Unix epoch — GrowHub internal ref)
// ============================================================
// Seconds since 2000-01-01 00:00:00 UTC. Not compatible with Unix
// epoch (1970-01-01). Use only for elapsed-time differences
// within GrowHub32. Do NOT export or compare against external
// timestamps (WiFi, NTP, ntfy.sh, fridge node).

unsigned long rtc_getGH2000Seconds() {
  RTCTime now;
  if (!rtc_readTime(&now)) {
    return 0;
  }

  return rtc_timeToGH2000Seconds(&now);
}

unsigned long rtc_timeToGH2000Seconds(const RTCTime* time) {
  // Pure calendar-to-seconds conversion using uint64_t intermediate
  // to prevent any possibility of 32-bit overflow through year 2099.
  // Base year: 2000 (NOT 1970 — not Unix epoch).
  if (!time) return 0;
  if (time->year < 2000 || time->year > 2099) return 0;
  if (time->month < 1 || time->month > 12) return 0;
  if (time->hours > 23 || time->minutes > 59 || time->seconds > 59) return 0;

  uint8_t maxDay = daysInMonth(time->month, time->year);
  if (time->date < 1 || time->date > maxDay) return 0;

  uint64_t days = 0;
  for (uint16_t y = 2000; y < time->year; y++) {
    days += isLeapYear(y) ? 366ULL : 365ULL;
  }

  for (uint8_t m = 1; m < time->month; m++) {
    days += (uint64_t)daysInMonth(m, time->year);
  }

  days += (uint64_t)(time->date - 1);

  uint64_t totalSeconds = days * 86400ULL
                        + (uint64_t)time->hours * 3600ULL
                        + (uint64_t)time->minutes * 60ULL
                        + (uint64_t)time->seconds;

  if (totalSeconds > ULONG_MAX) return 0;
  return (unsigned long)totalSeconds;
}

// ============================================================
// v1.3: Night Mode Scheduling
// ============================================================

int rtc_minutesUntilNightMode() {
  RTCTime now;
  if (!rtc_readTime(&now)) {
    return -1;
  }

  int currentMinutes = (now.hours * 60) + now.minutes;
  int nightStartMinutes = NIGHT_MODE_START_HOUR * 60;
  int nightEndMinutes = NIGHT_MODE_END_HOUR * 60;

  if (currentMinutes >= nightStartMinutes || currentMinutes < nightEndMinutes) {
    return 0;
  }

  return nightStartMinutes - currentMinutes;
}

void rtc_checkSerialCommand() {
  // Non-blocking character-by-character serial parser.
  // Zero heap allocation — uses strtol() instead of sscanf().
  // Accumulates characters into a static buffer until newline.
  static char cmd[32];
  static uint8_t cmdLen = 0;
  static bool overflow = false;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (overflow) {
        Serial.print(F("[RTC] ERROR: Command too long (max "));
        Serial.print(sizeof(cmd) - 1);
        Serial.println(F(" characters)."));
        overflow = false;
        cmdLen = 0;
        continue;
      }

      if (cmdLen > 0) {
        cmd[cmdLen] = '\0';

        // Zero-allocation parser using strtol
        char* p = cmd;
        while (*p == ' ' || *p == '\t') p++;

        if (strncmp(p, "SETTIME", 7) != 0 || (p[7] != ' ' && p[7] != '\t' && p[7] != '\0')) {
          Serial.println(F("[RTC] ERROR: Unknown command. Format: SETTIME YYYY MM DD HH MM SS"));
          cmdLen = 0;
          continue;
        }
        p += 7;
        while (*p == ' ' || *p == '\t') p++;

        char* end;
        bool parseOk = true;
        long y  = strtol(p, &end, 10); if (end == p) parseOk = false; p = end; while (*p == ' ' || *p == '\t') p++;
        long mo = strtol(p, &end, 10); if (end == p) parseOk = false; p = end; while (*p == ' ' || *p == '\t') p++;
        long d  = strtol(p, &end, 10); if (end == p) parseOk = false; p = end; while (*p == ' ' || *p == '\t') p++;
        long h  = strtol(p, &end, 10); if (end == p) parseOk = false; p = end; while (*p == ' ' || *p == '\t') p++;
        long m  = strtol(p, &end, 10); if (end == p) parseOk = false; p = end; while (*p == ' ' || *p == '\t') p++;
        long s  = strtol(p, &end, 10); if (end == p) parseOk = false;

        if (!parseOk) {
          Serial.println(F("[RTC] ERROR: Format is: SETTIME YYYY MM DD HH MM SS"));
          Serial.println(F("[RTC] Example: SETTIME 2026 7 19 20 30 0"));
          cmdLen = 0;
          continue;
        }

        // Validate ranges
        if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
            h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
          Serial.println(F("[RTC] ERROR: Values out of range"));
          cmdLen = 0;
          continue;
        }

        uint8_t dow = computeDayOfWeek((uint16_t)y, (uint8_t)mo, (uint8_t)d);

        if (rtc_setTime((uint8_t)h, (uint8_t)m, (uint8_t)s,
                        (uint8_t)d, (uint8_t)mo, (uint16_t)y, dow)) {
          Serial.println(F("[RTC] Time set successfully!"));
          char timeStr[24];
          rtc_getTimeString(timeStr, sizeof(timeStr));
          Serial.print(F("[RTC] Now: "));
          Serial.println(timeStr);
        } else {
          Serial.println(F("[RTC] ERROR: Invalid date/time values. Check ranges."));
        }
        cmdLen = 0;
      }
    } else if (!overflow) {
      if (cmdLen < sizeof(cmd) - 1) {
        cmd[cmdLen++] = c;
      } else {
        overflow = true;
      }
    }
  }
}
