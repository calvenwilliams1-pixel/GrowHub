/*
   rtc_handler.h
   GrowHub32 - DS3231 Real-Time Clock Handler
   Version: 1.2.3
   Revision: Added rtc_getEpochSeconds() helper for cooldown persistence.
*/

#ifndef RTC_HANDLER_H
#define RTC_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
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
bool rtc_isNightMode();
float rtc_getTemperature();
bool rtc_isValid();
const char* rtc_getTimeString();

// Epoch seconds helper (approximate, sufficient for cooldown comparisons)
unsigned long rtc_getEpochSeconds();

#endif // RTC_HANDLER_H
