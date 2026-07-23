/*
   rtc_handler.h
   GrowHub32 - DS3231 Real-Time Clock Handler
   Version: 1.3.0
   Revision: Added rtc_minutesUntilNightMode() for calibration scheduling.
             Added rtc_checkSerialCommand() declaration.
             Changed rtc_getTimeString() to caller-provided buffer for reentrancy.

   DS3231 communicates over I2C at address 0x68.
   All time values are stored in BCD format internally.
   No DST or timezone adjustments are applied (per GH-NM-001).

   NIGHT MODE SCHEDULE:
   Night mode runs from NIGHT_MODE_START_HOUR:00 to NIGHT_MODE_END_HOUR:00
   the following day. The schedule spans midnight (e.g., 21:00 to 10:00).
   Defined in config.h — do not hardcode hours elsewhere.
*/

#ifndef RTC_HANDLER_H
#define RTC_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"

// DS3231 register addresses
#define DS3231_ADDRESS          0x68
#define DS3231_REG_SECONDS      0x00
#define DS3231_REG_MINUTES      0x01
#define DS3231_REG_HOURS        0x02
#define DS3231_REG_DAY          0x03
#define DS3231_REG_DATE         0x04
#define DS3231_REG_MONTH        0x05
#define DS3231_REG_YEAR         0x06
#define DS3231_REG_CONTROL      0x0E
#define DS3231_REG_STATUS       0x0F
#define DS3231_REG_TEMP_MSB     0x11
#define DS3231_REG_TEMP_LSB     0x12

// Time structure
struct RTCTime {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;      // 24-hour format
  uint8_t dayOfWeek;  // 1=Sunday, 7=Saturday
  uint8_t date;
  uint8_t month;
  uint16_t year;      // Full year (e.g., 2026)
  bool valid;
};

// Public API
bool rtc_init();
bool rtc_readTime(RTCTime* time);
bool rtc_setTime(uint8_t hours, uint8_t minutes, uint8_t seconds,
                 uint8_t date, uint8_t month, uint16_t year,
                 uint8_t dayOfWeek);

// Returns cached night-mode status. Cache is updated once per minute
// to minimize I2C traffic while maintaining accurate state.
bool rtc_isNightMode();

// Returns temperature in Celsius. Returns NAN on I2C failure.
float rtc_getTemperature();

// Performs a live RTC read to ensure validity reflects current hardware state.
bool rtc_isValid();

// Fills the provided buffer with a formatted time string (e.g., "YYYY-MM-DD HH:MM:SS").
// Caller must ensure bufferSize is at least 20 bytes. Safe and reentrant.
void rtc_getTimeString(char* buffer, size_t bufferSize);

// GH2000 seconds from RTC (calendar-aware, monotonic).
// Computes seconds since 2000-01-01 00:00:00 UTC — NOT Unix epoch (1970).
// Suitable ONLY for elapsed-time differences within GrowHub32.
// Do NOT compare against external timestamps. Returns 0 on RTC read failure.
unsigned long rtc_getGH2000Seconds();

// Convert a calendar date/time to GH2000 seconds (since 2000-01-01).
// Same algorithm as rtc_getGH2000Seconds() but accepts an explicit RTCTime
// instead of reading from hardware. Used by sd_logger for log purge age
// calculation without requiring a live RTC read per file.
// Returns 0 if the input pointer is NULL or the date is outside the valid
// range (year 2000–2099, month 1–12, date validated against actual month length).
unsigned long rtc_timeToGH2000Seconds(const RTCTime* time);

// ============================================================
// v1.3: Night Mode Scheduling Helper
// ============================================================
// Returns the number of minutes until night mode begins.
//   0  = night mode is currently active
//   >0 = minutes until night mode starts (e.g., 45 = 45 minutes from now)
//   -1 = RTC read failed (caller should treat as "cannot start calibration")
//
// Night mode spans midnight (NIGHT_MODE_START_HOUR to NIGHT_MODE_END_HOUR
// next day). This function handles the midnight boundary correctly.
//
// Primary consumer: adaptive.cpp (canStartCalibration) — prevents
// starting a 20-minute calibration within 30 minutes of night mode.
int rtc_minutesUntilNightMode();

// Check Serial Monitor for RTC time-setting command.
// Format: SETTIME YYYY MM DD HH MM SS
// Call from main loop. Non-blocking, zero heap allocation.
void rtc_checkSerialCommand();

#endif // RTC_HANDLER_H
