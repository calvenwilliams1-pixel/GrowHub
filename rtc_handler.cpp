/*
   rtc_handler.cpp
   GrowHub32 - DS3231 RTC Driver Implementation
   Version: 1.2.3
   Revision: Added rtc_getEpochSeconds() for cross-module timestamp consistency.
             Single-warning RTC failure message to prevent serial spam.
             Fixed Wire.requestFrom ambiguous overload with explicit casts.

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
  g_currentTime.valid = rtc_readTime(&g_currentTime);

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
  time->month     = bcdToDec(regs[5] & 0x1F);
  time->year      = 2000 + bcdToDec(regs[6]);  // DS3231 stores years 00-99

  // Validate ranges
  time->valid = (time->seconds < 60) &&
                (time->minutes < 60) &&
                (time->hours < 24) &&
                (time->date >= 1 && time->date <= 31) &&
                (time->month >= 1 && time->month <= 12) &&
                (time->year >= 2020 && time->year <= 2099);

  return time->valid;
}

bool rtc_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds,
                 uint8_t date, uint8_t month, uint16_t year,
                 uint8_t dayOfWeek) {
  // Validate inputs
  if (hours > 23 || minutes > 59 || seconds > 59) return false;
  if (date < 1 || date > 31 || month < 1 || month > 12) return false;
  if (year < 2000 || year > 2099) return false;
  if (dayOfWeek < 1 || dayOfWeek > 7) return false;

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
  // Evaluated directly from DS3231 without timezone or DST adjustment

  RTCTime now;
  if (!rtc_readTime(&now)) {
    // Print warning only once per boot to avoid serial spam
    static bool warned = false;
    if (!warned) {
      warned = true;
      Serial.println(F("[RTC] RTC unavailable - night mode disabled until reboot"));
    }
    return false;
  }

  uint8_t hour = now.hours;

  // Night mode: 21:00 through 23:59, and 00:00 through 09:59
  if (hour >= NIGHT_MODE_START_HOUR || hour < NIGHT_MODE_END_HOUR) {
    return true;
  }

  return false;
}

float rtc_getTemperature() {
  // DS3231 has an internal temperature sensor (+/- 3C accuracy)
  uint8_t msb = readRegister8(DS3231_REG_TEMP_MSB);
  uint8_t lsb = readRegister8(DS3231_REG_TEMP_LSB);

  // MSB is integer part (2's complement), LSB upper 2 bits are fractional (0.25C resolution)
  int8_t tempInt = (int8_t)msb;
  float tempFrac = (float)(lsb >> 6) * 0.25f;

  return (float)tempInt + tempFrac;
}

bool rtc_isValid() {
  return g_rtcInitialized && g_currentTime.valid;
}

const char* rtc_getTimeString() {
  static char timeStr[20];
  RTCTime now;
  if (rtc_readTime(&now)) {
    snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year, now.month, now.date,
             now.hours, now.minutes, now.seconds);
    return timeStr;
  }
  return "RTC ERROR";
}

unsigned long rtc_getEpochSeconds() {
  // Approximate epoch seconds from RTC.
  // Uses fixed 30-day months and ignores leap years — accurate enough
  // for a 10-minute cooldown comparison across reboots.
  RTCTime now;
  if (!rtc_readTime(&now)) {
    return 0;
  }

  return (unsigned long)now.year * 31536000UL +
         (unsigned long)now.month * 2592000UL +
         (unsigned long)now.date * 86400UL +
         (unsigned long)now.hours * 3600UL +
         (unsigned long)now.minutes * 60UL +
         (unsigned long)now.seconds;
}
