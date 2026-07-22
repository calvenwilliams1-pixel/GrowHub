/**
 * @file system_state.h
 * @brief GrowHub32 v1.3 — Central System State Structure
 * 
 * PURPOSE:
 * This file defines the single shared data structure that represents the
 * entire state of the GrowHub32 system. Every module reads from or writes
 * to this struct. By centralizing it in a dedicated header, we eliminate
 * the previous problem where the struct was defined inside the .ino file
 * and .cpp files relied on extern declarations that could fall out of sync.
 * 
 * WHY A SEPARATE HEADER:
 * Prior to v1.3, SystemState was defined in GrowHub32_MainNode.ino. This
 * meant that .cpp files only saw an extern declaration, and if fields were
 * added to the .ino but not reflected in every extern, the compiler wouldn't
 * catch the mismatch — producing silent memory corruption. Moving the struct
 * to a header ensures every compilation unit sees the exact same definition.
 * 
 * OWNERSHIP:
 * - sensors.cpp writes: currentTemp, currentHumidity, currentCO2,
 *                       lastKnownGood*, tempSensorFault, humiditySensorFault,
 *                       co2SensorFault, sensorFaultTimer
 * - automation.cpp writes: hoHActive, airAssistActive, exhaustFanActive,
 *                          compressorActive, nightModeActive, calibrationActive,
 *                          currentDutyCycle, pidOutput, setpointHumidity, currentBand
 * - relay_manager.cpp writes: hoHActive, airAssistActive, exhaustFanActive,
 *                             compressorActive (under mutex)
 * - network.cpp writes: wifiConnected, apModeActive, lastNTFYAlert,
 *                       fridgeHeartbeatLost, fridgeLastSequence, fridgeTemp
 * - web_ui.cpp / sd_logger.cpp: read-only access
 * 
 * THREAD SAFETY:
 * This struct is accessed from multiple FreeRTOS tasks:
 *   - Main Arduino loop task (Core 1): sensors, automation, UI, logging
 *   - ESP-NOW receive callback (Core 0, WiFi task context): fridge telemetry
 *   - WiFi event callbacks (system task context): connection state
 * 
 * All cross-task reads and writes MUST be wrapped with:
 *   portENTER_CRITICAL(&g_stateMux);
 *   // ... access g_systemState fields ...
 *   portEXIT_CRITICAL(&g_stateMux);
 * 
 * g_stateMux is defined as a portMUX_TYPE in network.cpp and declared
 * extern wherever needed.
 * 
 * FIELD ADDITION RULES:
 * - New fields go at the bottom of the struct (preserves binary layout
 *   of existing fields for any serialization/EEPROM code)
 * - Always initialize new fields in GrowHub32_MainNode.ino
 * - Update the ownership comments above when adding fields
 * 
 * v1.3 ADDITIONS:
 *   - currentDutyCycle: Final duty cycle sent to relays (0.0–100.0)
 *   - pidOutput: Raw PID output before duty cycle clamping
 *   - setpointHumidity: Active PID target humidity (changes per band)
 *   - currentBand: Active temperature band index (0=18-21, 1=21-24, 2=24-27, 3=27-30)
 */

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // ================================================================
    // Current Sensor Readings
    // Updated by: sensors.cpp (sensors_poll)
    // ================================================================
    float    currentTemp;              // Latest SCD40 temperature (°C)
    float    currentHumidity;          // Latest SCD40 relative humidity (%)
    uint16_t currentCO2;               // Latest SCD40 CO2 (ppm)

    // ================================================================
    // Last Known Good Values
    // Updated by: sensors.cpp
    // Used as fallback when sensor fault is active. These are the last
    // successfully validated readings before the fault occurred.
    // ================================================================
    float    lastKnownGoodTemp;
    float    lastKnownGoodHumidity;
    uint16_t lastKnownGoodCO2;

    // ================================================================
    // Sensor Fault Flags
    // Updated by: sensors.cpp, GrowHub32_MainNode.ino (checkSensorFaults)
    // Set true when a sensor has failed to produce valid data for
    // SENSOR_FAULT_TIMEOUT_MS (10 seconds). Automation uses lastKnownGood
    // values while faults are active.
    // ================================================================
    bool         tempSensorFault;
    bool         humiditySensorFault;
    bool         co2SensorFault;
    unsigned long sensorFaultTimer;    // millis() when first fault detected

    // ================================================================
    // System Mode Flags
    // Updated by: automation.cpp (nightModeActive via rtc_isNightMode),
    //             adaptive.cpp (calibrationActive)
    // ================================================================
    bool nightModeActive;              // True between 21:00–10:00 (noise ordinance)
    bool calibrationActive;            // True during active calibration sequence

    // ================================================================
    // Actuator States
    // Updated by: relay_manager.cpp (under g_stateMux), automation.cpp
    // These reflect the commanded state, not necessarily the electrical
    // state (relay_manager may block commands due to cooldown/cycle limits).
    // ================================================================
    bool hoHActive;                    // HOH humidifier relay
    bool airAssistActive;              // Air Assist valve relay
    bool exhaustFanActive;             // Exhaust fan relay
    bool compressorActive;             // Air compressor relay

    // ================================================================
    // Network & Connectivity
    // Updated by: network.cpp (WiFi event callbacks, under g_stateMux)
    // ================================================================
    bool         wifiConnected;        // Station mode connected to router
    bool         apModeActive;         // Access Point fallback mode active
    unsigned long lastNTFYAlert;       // millis() of last push notification sent

    // ================================================================
    // Fridge Node Telemetry (ESP-NOW)
    // Updated by: network.cpp (onESPNOWReceive callback, under g_stateMux)
    // Read by: web_ui.cpp for dashboard display
    // ================================================================
    bool     fridgeHeartbeatLost;      // True if >60s since last valid packet
    uint16_t fridgeLastSequence;       // Last received sequence number
    float    fridgeTemp;               // Last received fridge temperature (°C)

    // ================================================================
    // v1.3: PID Controller State
    // Updated by: automation.cpp (executeHumidityControl)
    // Read by: web_ui.cpp (dashboard display), sd_logger.cpp (data logging)
    // ================================================================
    float  currentDutyCycle;           // Final duty cycle 0.0–100.0 (after clamping)
    float  pidOutput;                  // Raw PID output before duty cycle conversion
    float  setpointHumidity;           // Active PID target (changes per temperature band)
    uint8_t currentBand;               // Active temperature band index (0–3)

} SystemState;

// Single global instance — defined in GrowHub32_MainNode.ino
extern SystemState g_systemState;

#endif // SYSTEM_STATE_H
