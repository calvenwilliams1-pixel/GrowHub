/*
   sensors.cpp
   GrowHub32 - SCD40 Sensor Driver Implementation
   Version: 1.2.1

   SCD40 communicates over I2C at address 0x62.
   Measurement interval in periodic mode is approximately 5 seconds.
   Data ready must be checked before each read attempt.

   CRC-8 polynomial used by SCD40: 0x31 (x^8 + x^5 + x^4 + 1)
*/

#include "sensors.h"

static SCD40Data g_scd40Data;
static bool g_scd40Initialized = false;
static unsigned long g_lastDataReadyCheck = 0;
static uint8_t g_consecutiveFailures = 0;

// CRC-8 lookup table for SCD40 (polynomial 0x31, init 0xFF)
static const uint8_t crc8_table[256] = {
  0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
  0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
  0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
  0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
  0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
  0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
  0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
  0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
  0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
  0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
  0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
  0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
  0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
  0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
  0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
  0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
  0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
  0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
  0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
  0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
  0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
  0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
  0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
  0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
  0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
  0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
  0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
  0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
  0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
  0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
  0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
  0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

// --- Private Helper Functions ---

static uint8_t computeCRC8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

static bool sendSCD40Command(uint16_t command) {
  Wire.beginTransmission(SCD40_I2C_ADDRESS);
  Wire.write(command >> 8);
  Wire.write(command & 0xFF);
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    Serial.print(F("[SCD40] I2C transmit error for command 0x"));
    Serial.println(command, HEX);
    return false;
  }
  return true;
}

static bool readSCD40Words(uint8_t count, uint16_t* buffer) {
  // Each word is 3 bytes: 2 data + 1 CRC
  uint8_t bytesNeeded = count * 3;

  Wire.requestFrom((uint8_t)SCD40_I2C_ADDRESS, bytesNeeded);
  if (Wire.available() != bytesNeeded) {
    Serial.println(F("[SCD40] Read error: insufficient bytes"));
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    uint8_t dataHigh = Wire.read();
    uint8_t dataLow = Wire.read();
    uint8_t crcReceived = Wire.read();

    uint8_t crcData[2] = {dataHigh, dataLow};
    uint8_t crcCalculated = computeCRC8(crcData, 2);

    if (crcReceived != crcCalculated) {
      Serial.print(F("[SCD40] CRC mismatch at word "));
      Serial.print(i);
      Serial.print(F(": got 0x"));
      Serial.print(crcReceived, HEX);
      Serial.print(F(", expected 0x"));
      Serial.println(crcCalculated, HEX);
      return false;
    }

    buffer[i] = ((uint16_t)dataHigh << 8) | dataLow;
  }

  return true;
}

static void serialNumberToStr(uint64_t serial, char* buffer, size_t bufLen) {
  snprintf(buffer, bufLen, "0x%04X%04X%04X",
           (uint16_t)(serial >> 32),
           (uint16_t)(serial >> 16),
           (uint16_t)(serial & 0xFFFF));
}

// --- Public API Implementation ---

bool sensors_init() {
  Serial.println(F("[SCD40] Initializing sensor..."));

  // Initialize data structure
  g_scd40Data.co2_ppm = 0;
  g_scd40Data.temperature_c = 0.0f;
  g_scd40Data.humidity_pct = 0.0f;
  g_scd40Data.lastReadTimestamp = 0;
  g_scd40Data.valid = false;
  g_consecutiveFailures = 0;

  // Check if device is present on I2C bus
  Wire.beginTransmission(SCD40_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[SCD40] Device not found on I2C bus. Check wiring."));
    return false;
  }
  Serial.println(F("[SCD40] Device found on I2C bus."));

  // Stop any ongoing measurement
  sendSCD40Command(SCD40_CMD_STOP_PERIODIC_MEASUREMENT);
  delay(500);

  // Read serial number for verification
  sendSCD40Command(SCD40_CMD_GET_SERIAL_NUMBER);
  delay(1);

  uint16_t serialWords[3];
  if (readSCD40Words(3, serialWords)) {
    uint64_t serial = ((uint64_t)serialWords[0] << 32) |
                      ((uint64_t)serialWords[1] << 16) |
                      serialWords[2];
    char serialStr[32];
    serialNumberToStr(serial, serialStr, sizeof(serialStr));
    Serial.print(F("[SCD40] Serial Number: "));
    Serial.println(serialStr);
  } else {
    Serial.println(F("[SCD40] Failed to read serial number"));
    return false;
  }

  // Set sensor altitude (approximate sea level for most grow setups)
  sendSCD40Command(SCD40_CMD_SET_SENSOR_ALTITUDE);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(computeCRC8((const uint8_t*)"\x00\x00", 2));
  Wire.endTransmission();
  delay(1);

  // Start periodic measurement
  if (!sendSCD40Command(SCD40_CMD_START_PERIODIC_MEASUREMENT)) {
    Serial.println(F("[SCD40] Failed to start periodic measurement"));
    return false;
  }

  Serial.println(F("[SCD40] Periodic measurement started. First reading available in ~5s."));
  g_scd40Initialized = true;
  g_lastDataReadyCheck = millis();

  return true;
}

bool sensors_isDataReady() {
  if (!g_scd40Initialized) return false;

  // Only check data ready every 500ms to avoid I2C congestion
  if (millis() - g_lastDataReadyCheck < 500) {
    // Return cached ready state
    return g_scd40Data.valid;
  }
  g_lastDataReadyCheck = millis();

  if (!sendSCD40Command(SCD40_CMD_GET_DATA_READY_STATUS)) {
    return false;
  }

  uint16_t status[1];
  if (!readSCD40Words(1, status)) {
    return false;
  }

  // Mask with 0x07FF per datasheet (upper 5 bits are reserved)
  uint16_t maskedStatus = status[0] & 0x07FF;

  if (maskedStatus == 0x0000) {
    return false;  // Data not ready
  }

  // Data is ready - treat any non-zero low 11 bits as ready signal
  return true;
}

void sensors_poll() {
  if (!g_scd40Initialized) {
    return;
  }

  if (!sensors_isDataReady()) {
    // Data not ready yet (SCD40 produces new data every ~5s)
    g_consecutiveFailures++;
    if (g_consecutiveFailures > 10) {
      // Over 10 polling cycles (20+ seconds) with no data - flag fault
      g_scd40Data.valid = false;
    }
    return;
  }

  // Read measurement
  if (!sendSCD40Command(SCD40_CMD_READ_MEASUREMENT)) {
    g_consecutiveFailures++;
    return;
  }

  uint16_t rawData[3];
  if (!readSCD40Words(3, rawData)) {
    g_consecutiveFailures++;
    return;
  }

  // Parse data
  // CO2: raw value is ppm
  uint16_t co2 = rawData[0];

  // Temperature: raw = (175 * T[C]) / 65536 - 45
  float temp = -45.0f + 175.0f * ((float)rawData[1] / 65536.0f);

  // Humidity: raw = (100 * RH[%]) / 65536
  float hum = 100.0f * ((float)rawData[2] / 65536.0f);

  // Range validation
  bool co2Valid = (co2 >= 0 && co2 <= 40000);
  bool tempValid = (temp >= -10.0f && temp <= 60.0f);
  bool humValid = (hum >= 0.0f && hum <= 100.0f);

  if (co2Valid && tempValid && humValid) {
    g_scd40Data.co2_ppm = co2;
    g_scd40Data.temperature_c = temp;
    g_scd40Data.humidity_pct = hum;
    g_scd40Data.lastReadTimestamp = millis();
    g_scd40Data.valid = true;
    g_consecutiveFailures = 0;

    // Update system state
    g_systemState.currentTemp = temp;
    g_systemState.currentHumidity = hum;
    g_systemState.currentCO2 = co2;

    // Update last known good values
    g_systemState.lastKnownGoodTemp = temp;
    g_systemState.lastKnownGoodHumidity = hum;
    g_systemState.lastKnownGoodCO2 = co2;

    Serial.print(F("[SCD40] CO2: "));
    Serial.print(co2);
    Serial.print(F(" ppm, Temp: "));
    Serial.print(temp, 1);
    Serial.print(F(" C, Hum: "));
    Serial.print(hum, 1);
    Serial.println(F(" %"));
  } else {
    g_scd40Data.valid = false;
    g_consecutiveFailures++;
    Serial.println(F("[SCD40] Data out of valid range:"));
    Serial.print(F("  CO2=")); Serial.print(co2); Serial.print(co2Valid ? F(" OK, ") : F(" BAD, "));
    Serial.print(F("Temp=")); Serial.print(temp); Serial.print(tempValid ? F(" OK, ") : F(" BAD, "));
    Serial.print(F("Hum=")); Serial.print(hum); Serial.println(humValid ? F(" OK") : F(" BAD"));
  }
}

bool sensors_isTemperatureValid() {
  if (!g_scd40Data.valid) return false;

  // Last reading must be within SENSOR_FAULT_TIMEOUT_MS
  if (millis() - g_scd40Data.lastReadTimestamp > SENSOR_FAULT_TIMEOUT_MS) {
    return false;
  }

  return (g_scd40Data.temperature_c >= -10.0f && g_scd40Data.temperature_c <= 60.0f);
}

bool sensors_isHumidityValid() {
  if (!g_scd40Data.valid) return false;

  if (millis() - g_scd40Data.lastReadTimestamp > SENSOR_FAULT_TIMEOUT_MS) {
    return false;
  }

  return (g_scd40Data.humidity_pct >= 0.0f && g_scd40Data.humidity_pct <= 100.0f);
}

bool sensors_isCO2Valid() {
  if (!g_scd40Data.valid) return false;

  if (millis() - g_scd40Data.lastReadTimestamp > SENSOR_FAULT_TIMEOUT_MS) {
    return false;
  }

  return (g_scd40Data.co2_ppm >= 0 && g_scd40Data.co2_ppm <= 40000);
}

float sensors_getTemperature() {
  return g_scd40Data.temperature_c;
}

float sensors_getHumidity() {
  return g_scd40Data.humidity_pct;
}

uint16_t sensors_getCO2() {
  return g_scd40Data.co2_ppm;
}

bool sensors_hasActiveFault() {
  return (!sensors_isTemperatureValid() ||
          !sensors_isHumidityValid() ||
          !sensors_isCO2Valid());
}

void sensors_clearFaults() {
  g_consecutiveFailures = 0;
}
