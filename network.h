/*
   network.h
   GrowHub32 - Wireless Networking & Alert Subsystem
   Version: 1.2.1
   Revision: Added FreeRTOS concurrency-safe accessors for fridge data.
             Added ntfy.sh SSL client support.
             Added watchdog feed during blocking WiFi connect.

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
#pragma pack(push, 1)
struct FridgePacket {
  uint16_t sequenceNumber;
  float temperature;
  uint16_t crc16;
};
#pragma pack(pop)

// Public API
bool network_init();
void network_checkConnection();
void network_checkFridgeHeartbeat();
void network_sendAlert(const char* title, const char* message);
bool network_isWiFiConnected();
bool network_isAPMode();
String network_getIPAddress();

// FreeRTOS-safe accessors for cross-task fridge data
// Use these instead of reading g_systemState directly from non-loop tasks
float network_getFridgeTemp();
bool network_isFridgeHeartbeatLost();
uint16_t network_getFridgeLastSequence();

// ESP-NOW callback (runs in WiFi task context)
void onESPNOWReceive(const uint8_t* mac, const uint8_t* incomingData, int len);

// CRC-16 implementation for ESP-NOW validation
uint16_t network_crc16(const uint8_t* data, size_t len);

#endif // NETWORK_H
