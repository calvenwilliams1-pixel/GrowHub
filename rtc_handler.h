/*
   rtc_handler.h
   GrowHub32 - DS3231 Real-Time Clock Handler
   Version: 1.2.6
   Revision: Changed rtc_getTimeString() to caller-provided buffer for reentrancy.
             Added clarifying comments on caching and validity behavior.
             Added rtc_checkSerialCommand() declaration.

   DS3231 communicates over I2C at address 0x68.
   All time values are stored in BCD format internally.
   No DST or timezone adjustments are applied (per GH-NM-001).
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
// Uses a full rtc_readTime() call (not just the status register) so that
// I2C communication failures are properly detected as invalid.
bool rtc_isValid();

// Fills the provided buffer with a formatted time string (e.g., "YYYY-MM-DD HH:MM:SS").
// Caller must ensure bufferSize is at least 20 bytes. Safe and reentrant.
void rtc_getTimeString(char* buffer, size_t bufferSize);

// Epoch seconds from RTC (calendar-aware, monotonic).
// Computes a pseudo-epoch offset from 2000-01-01 00:00:00 UTC using actual
// month lengths and leap year correction. Calendar-correct and monotonic
// for normal RTC progression. Returns 0 on RTC read failure.
// Suitable ONLY for elapsed-time differences on the same device.
unsigned long rtc_getEpochSeconds();

// Convert a calendar date/time to monotonic pseudo-epoch seconds.
// Same algorithm as rtc_getEpochSeconds() but accepts an explicit RTCTime
// instead of reading from hardware. Used by sd_logger for log purge age
// calculation without requiring a live RTC read per file.
// Returns 0 if the input pointer is NULL or the date is outside the valid
// range (year 2000–2099, month 1–12, date validated against actual month length).
unsigned long rtc_timeToEpoch(const RTCTime* time);

// Check Serial Monitor for RTC time-setting command.
// Format: SETTIME YYYY MM DD HH MM SS
// Example: SETTIME 2026 7 19 20 30 0
// Call from main loop. Non-blocking, zero heap allocation.
void rtc_checkSerialCommand();

#endif // RTC_HANDLER_H