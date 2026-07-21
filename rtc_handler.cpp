/*
   rtc_handler.cpp
   GrowHub32 - DS3231 RTC Driver Implementation
   Version: 1.2.6
   Revision: Changed rtc_getTimeString() to caller-provided buffer for reentrancy.
             Fixed rtc_setTime() and rtc_readTime() to validate dates against actual
             month lengths using daysInMonth().
             Rewrote rtc_checkSerialCommand() with zero-allocation non-blocking parser,
             overflow protection, and correct Zeller's Congruence weekday computation.
             Added century bit documentation comment.
             Removed redundant g_currentTime.valid assignment in rtc_init().
             Fixed missing time->date assignment in rtc_readTime() (post-review fix).

   DS3231 communicates over I2C at address 0x68.
   All time values are stored in BCD format internally.
   No DST or timezone adjustments are applied (per GH-NM-001).
*/

#include "rtc_handler.h"

static RTCTime g_currentTime;
static bool g_rtcInitialized = false;

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

  // Initial time read — valid field is set inside rtc_readTime()
  rtc_readTime(&g_currentTime);

  if (g_currentTime.valid) {
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
  return true;
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
    // 12-hour mode: convert to 24h
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
    // 24-hour mode (preferred)
    time->hours = bcdToDec(hourReg & 0x3F);
  }

  time->dayOfWeek = bcdToDec(regs[3] & 0x07);
  time->date      = bcdToDec(regs[4] & 0x3F);

  // Century bit (bit 7 of month register) ignored; GrowHub32 supports 2000–2099 only.
  time->month     = bcdToDec(regs[5] & 0x1F);
  time->year      = 2000 + bcdToDec(regs[6]);  // DS3231 stores years 00-99

  // Validate ranges using actual month length
  bool monthValid = (time->month >= 1 && time->month <= 12);
  uint8_t maxDay = monthValid ? daysInMonth(time->month, time->year) : 0;

  time->valid = (time->seconds < 60) &&
                (time->minutes < 60) &&
                (time->hours < 24) &&
                monthValid &&
                (time->date >= 1 && time->date <= maxDay) &&
                (time->year >= 2020 && time->year <= 2099);

  return time->valid;
}

bool rtc_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds,
                 uint8_t date, uint8_t month, uint16_t year,
                 uint8_t dayOfWeek) {
  // Validate inputs
  if (hours > 23 || minutes > 59 || seconds > 59) return false;
  if (dayOfWeek < 1 || dayOfWeek > 7) return false;
  if (year < 2000 || year > 2099) return false;
  if (month < 1 || month > 12) return false;

  // Validate date against actual month length
  uint8_t maxDay = daysInMonth(month, year);
  if (date < 1 || date > maxDay) return false;

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_REG_SECONDS);
  Wire.write(decToBcd(seconds));
  Wire.write(decToBcd(minutes));
  Wire.write(decToBcd(hours));          // 24-hour mode
  Wire.write(decToBcd(dayOfWeek));
  Wire.write(decToBcd(date));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd((uint8_t)(year - 2000)));

  if (Wire.endTransmission() != 0) {
    Serial.println(F("[RTC] Failed to set time"));
    return false;
  }

  // Clear the Oscillator Stop Flag (OSF) since we just set valid time
  uint8_t status = readRegister8(DS3231_REG_STATUS);
  if (status & 0x80) {
    writeRegister8(DS3231_REG_STATUS, status & 0x7F);
    Serial.println(F("[RTC] Oscillator Stop Flag cleared after time set"));
  }

  // Refresh cached state immediately
  rtc_readTime(&g_currentTime);

  Serial.print(F("[RTC] Time set to: "));
  Serial.print(hours); Serial.print(':');
  if (minutes < 10) Serial.print('0');
  Serial.print(minutes); Serial.print(':');
  if (seconds < 10) Serial.print('0');
  Serial.println(seconds);

  return true;
}

bool rtc_isNightMode() {
  // GH-NM-001: Night mode 9:00 PM (21:00) to 10:00 AM (10:00)
  // Evaluated directly from DS3231 without timezone or DST adjustment.
  // Result cached for 60 seconds to reduce I2C bus traffic.

  static bool cachedNightMode = false;
  static unsigned long lastCheck = 0;
  static uint8_t lastHour = 255;  // 255 forces first read on boot
  static bool warned = false;     // Unified scope — single declaration

  // Return cached result if checked within the last 60 seconds
  if (millis() - lastCheck < 60000UL && lastHour != 255) {
    return cachedNightMode;
  }
  lastCheck = millis();

  RTCTime now;
  if (!rtc_readTime(&now)) {
    if (!warned) {
      warned = true;
      Serial.println(F("[RTC] RTC unavailable - night mode disabled until recovery"));
    }
    lastHour = 0;
    lastCheck = 0; // Force immediate retry on next call instead of waiting 60s
    return false;
  }

  // Clear warning flag on successful read so future failures re-trigger
  warned = false;
  lastHour = now.hours;

  cachedNightMode = (now.hours >= NIGHT_MODE_START_HOUR || now.hours < NIGHT_MODE_END_HOUR);
  return cachedNightMode;
}

float rtc_getTemperature() {
  // DS3231 has an internal temperature sensor (+/- 3C accuracy).
  // Reads both bytes atomically to prevent torn reads.

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(DS3231_REG_TEMP_MSB);
  if (Wire.endTransmission() != 0) {
    return NAN;
  }

  // Request both bytes in a single I2C transaction for atomicity
  Wire.requestFrom((uint8_t)DS3231_ADDRESS, (uint8_t)2);
  if (Wire.available() < 2) {
    return NAN;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  // MSB is integer part (2's complement), LSB upper 2 bits are fractional (0.25C resolution)
  int8_t tempInt = (int8_t)msb;
  float tempFrac = (float)(lsb >> 6) * 0.25f;

  return (float)tempInt + tempFrac;
}

bool rtc_isValid() {
  // Perform a live RTC read to ensure validity reflects current hardware state.
  // Uses a full rtc_readTime() so that I2C communication failures are detected.
  if (!g_rtcInitialized) return false;

  RTCTime now;
  return rtc_readTime(&now);
}

void rtc_getTimeString(char* buffer, size_t bufferSize) {
  // Fills caller-provided buffer with formatted time string.
  // Safe and reentrant — no static buffer.
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

unsigned long rtc_getEpochSeconds() {
  // Calendar-aware pseudo-epoch calculation.
  // Computes seconds since 2000-01-01 00:00:00 UTC using actual month lengths
  // and leap year correction. Calendar-correct and monotonic for normal RTC
  // progression. Returns 0 on RTC read failure.
  // Suitable ONLY for elapsed-time differences on the same device.

  RTCTime now;
  if (!rtc_readTime(&now)) {
    return 0;
  }

  return rtc_timeToEpoch(&now);
}

unsigned long rtc_timeToEpoch(const RTCTime* time) {
  // Pure calendar-to-epoch conversion.
  // Same algorithm as rtc_getEpochSeconds() but operates on a provided struct
  // instead of reading hardware. Used by sd_logger for log purge age calculation
  // without requiring a live RTC read per file.
  // Returns 0 if the input pointer is NULL or the date is outside the valid
  // range (year 2000–2099, month 1–12, date validated against actual month length).

  if (!time) return 0;
  if (time->year < 2000 || time->year > 2099) return 0;
  if (time->month < 1 || time->month > 12) return 0;

  // Validate date against actual month length
  uint8_t maxDay = daysInMonth(time->month, time->year);
  if (time->date < 1 || time->date > maxDay) return 0;

  // Days from 2000 to current year (full years)
  unsigned long days = 0;
  for (uint16_t y = 2000; y < time->year; y++) {
    days += isLeapYear(y) ? 366UL : 365UL;
  }

  // Days from January to current month (full months in current year)
  for (uint8_t m = 1; m < time->month; m++) {
    days += (unsigned long)daysInMonth(m, time->year);
  }

  // Days in current month (day - 1 because current day is not yet complete)
  days += (unsigned long)(time->date - 1);

  // Convert days to seconds and add time-of-day
  unsigned long totalSeconds = days * 86400UL;
  totalSeconds += (unsigned long)time->hours * 3600UL;
  totalSeconds += (unsigned long)time->minutes * 60UL;
  totalSeconds += (unsigned long)time->seconds;

  return totalSeconds;
}

void rtc_checkSerialCommand() {
  // Non-blocking character-by-character serial parser.
  // Accumulates characters into a static buffer until newline.
  // No heap allocation, no blocking timeout — reads only what's available.
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

        int y, mo, d, h, m, s;
        int params = sscanf(cmd, "SETTIME %d %d %d %d %d %d", &y, &mo, &d, &h, &m, &s);

        if (params == 6) {
          uint8_t dow = computeDayOfWeek((uint16_t)y, (uint8_t)mo, (uint8_t)d);

          if (rtc_setTime((uint8_t)h, (uint8_t)m, (uint8_t)s, (uint8_t)d, (uint8_t)mo, (uint16_t)y, dow)) {
            Serial.println(F("[RTC] Time set successfully!"));

            char timeStr[24];
            rtc_getTimeString(timeStr, sizeof(timeStr));
            Serial.print(F("[RTC] Now: "));
            Serial.println(timeStr);
          } else {
            Serial.println(F("[RTC] ERROR: Invalid date/time values. Check ranges."));
          }
        } else {
          Serial.println(F("[RTC] ERROR: Format is: SETTIME YYYY MM DD HH MM SS"));
          Serial.println(F("[RTC] Example: SETTIME 2026 7 19 20 30 0"));
        }
        cmdLen = 0;
      }
    } else if (!overflow) {
      // Append character if buffer has space
      if (cmdLen < sizeof(cmd) - 1) {
        cmd[cmdLen++] = c;
      } else {
        overflow = true;
      }
    }
  }
}