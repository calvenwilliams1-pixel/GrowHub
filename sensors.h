/*
   sensors.h
   GrowHub32 - SCD40 CO2/Temperature/Humidity Sensor Driver
   Version: 1.2.1

   SCD40 communicates over I2C at address 0x62.
   Measurement interval in periodic mode is approximately 5 seconds.
   Data ready must be checked before each read attempt.

   CRC-8 polynomial used by SCD40: 0x31 (x^8 + x^5 + x^4 + 1)
*/

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// SCD40 command set (Sensirion SCD4x datasheet)
#define SCD40_CMD_START_PERIODIC_MEASUREMENT  0x21B1
#define SCD40_CMD_STOP_PERIODIC_MEASUREMENT   0x3F86
#define SCD40_CMD_READ_MEASUREMENT            0xEC05
#define SCD40_CMD_GET_DATA_READY_STATUS       0xE4B8
#define SCD40_CMD_SET_TEMPERATURE_OFFSET      0x241D
#define SCD40_CMD_GET_TEMPERATURE_OFFSET      0x2318
#define SCD40_CMD_SET_SENSOR_ALTITUDE         0x2427
#define SCD40_CMD_GET_SENSOR_ALTITUDE         0x2326
#define SCD40_CMD_SET_AMBIENT_PRESSURE        0xE000
#define SCD40_CMD_PERFORM_FORCED_RECAL        0x362F
#define SCD40_CMD_SET_AUTOSELF_CALIBRATION    0x2416
#define SCD40_CMD_GET_AUTOSELF_CALIBRATION    0x2316
#define SCD40_CMD_START_LOW_POWER_MEASUREMENT 0x21AC
#define SCD40_CMD_GET_SERIAL_NUMBER           0x3682
#define SCD40_CMD_PERFORM_SELF_TEST           0x3639
#define SCD40_CMD_PERSIST_SETTINGS            0x3615
#define SCD40_CMD_REINIT                      0x3646

// Data ready status response values
#define SCD40_STATUS_DATA_NOT_READY   0x0000
#define SCD40_STATUS_DATA_READY       0xFFFF

// SCD40 measurement data structure
struct SCD40Data {
  uint16_t co2_ppm;
  float temperature_c;
  float humidity_pct;
  unsigned long lastReadTimestamp;
  bool valid;
};

// Public API
bool sensors_init();
void sensors_poll();
bool sensors_isDataReady();

// Individual sensor value accessors
bool sensors_isTemperatureValid();
bool sensors_isHumidityValid();
bool sensors_isCO2Valid();
float sensors_getTemperature();
float sensors_getHumidity();
uint16_t sensors_getCO2();

// Sensor fault management
bool sensors_hasActiveFault();
void sensors_clearFaults();

#endif // SENSORS_H
