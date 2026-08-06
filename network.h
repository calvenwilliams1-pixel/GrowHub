/*
   network.h
   GrowHub32 - Wireless Networking & Alert Subsystem
   Version: 1.2.4
   Revision: Added mDNS guard flag. Added g_fridgePacketEverReceived.
             Added g_wifiConnectedTime for AP stability tracking.
             Documented mutex requirements for all cross-task state.

   Handles:
   - WiFi station mode connection (GH-NET-001)
   - AP fallback mode (GH-NET-002)
   - ntfy.sh push notifications (GH-NET-005)
   - ESP-NOW fridge node listener stub (GH-NET-003, GH-NET-004)
*/

#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include "config.h"

// ESP-NOW fridge packet structure (per SRS Appendix B)
// v1.4 fridge-bridge: expanded from 8 to 13 bytes. Backward-compatible
// on receive — old 8-byte packets accepted with NAN/default fallback.
#pragma pack(push, 1)
struct FridgePacket {
  uint16_t sequenceNumber;  // offset 0
  float temperature;        // offset 2  — validated -40..+85°C on receive
  float humidity;           // offset 6  — validated 0..100% on receive (v1.4)
  uint8_t doorState;        // offset 10 — 0=closed, 1=open (v1.4)
  uint16_t crc16;           // offset 11 — CRC-16-CCITT over bytes 0–10
};
#pragma pack(pop)

// Public API
bool network_init();
void network_checkConnection();
void network_checkFridgeHeartbeat();
void network_sendAlert(const char* title, const char* message);
void network_sendAlivePing();
void network_fetchWeather();
bool network_isWiFiConnected();
bool network_isAPMode();
String network_getIPAddress();

// FreeRTOS-safe accessors for cross-task fridge data
// Use these instead of reading g_systemState directly from non-loop tasks
float network_getFridgeTemp();
float network_getFridgeHumidity();
bool network_isFridgeDoorOpen();
uint8_t network_getActiveAlerts();
bool network_isFridgeHeartbeatLost();
uint16_t network_getFridgeLastSequence();
// ESP-NOW callback (runs in WiFi task context)
void onESPNOWReceive(const uint8_t* mac, const uint8_t* incomingData, int len);

// CRC-16 implementation for ESP-NOW validation
uint16_t network_crc16(const uint8_t* data, size_t len);

#endif // NETWORK_H
