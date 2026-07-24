/*
   network.cpp
   GrowHub32 - Wireless Networking & Alert Subsystem Implementation
   Version: 1.3.0
   Revision: Added system_state.h include. Centralized network credentials.
             Mutex-protected lastNTFYAlert rate-limit check.
             Wraparound-aware sequence number comparison.
             AP stability timer uses g_wifiConnectedTime.
             Mutex-protected heartbeat reads in network_checkFridgeHeartbeat.
             mDNS guarded from repeated begin() calls.
             Immediate AP fallback on boot failure.
             Mutex-protected g_wifiDisconnectStart in event callback.
             Distinguish "never received" from "heartbeat lost".
             lastNTFYAlert updated only after successful HTTP send.
             Mutex-protected reads in network_checkConnection.
             Mutex-protected g_mdnsStarted access.
             Fixed switch scoping bug in onWiFiEvent GOT_IP case.
             Fixed mDNS race condition — check+claim inside single critical section.
             Failed mDNS start reverts g_mdnsStarted flag.
             Added MDNS.addService in AP fallback block for consistency.
             Mutex-protected g_wifiDisconnectStart writes in network_checkConnection.

   WiFi credentials are configured below per GH-NET-001.
   Alert endpoint is configured below per GH-NET-005.
   ESP-NOW listens for Remote Fridge Node packets.
*/

#include "network.h"
#include "sensors.h"
#include "safety.h"
#include "system_state.h"
#include <ESPmDNS.h>

// ============================================================
// Network Credentials (private — do not commit to version control)
// ============================================================
#define WIFI_SSID                       "DSLAP"
#define WIFI_PASSWORD                   "12345678910"
#define AP_SSID                         "GrowHub32_AP"
#define AP_PASSWORD                     "growhub123"

// Static IP configuration
#define STATIC_IP_OCTET_1               10
#define STATIC_IP_OCTET_2               0
#define STATIC_IP_OCTET_3               0
#define STATIC_IP_OCTET_4               20
#define GATEWAY_OCTET_1                 10
#define GATEWAY_OCTET_2                 0
#define GATEWAY_OCTET_3                 0
#define GATEWAY_OCTET_4                 1

// ntfy.sh push notification endpoint
#define NTFY_ENDPOINT                   "ntfy.sh/growhub32_alerts"

// ============================================================
// FreeRTOS Concurrency Guard
// ============================================================

// Protects g_systemState access between:
//   - Main Arduino loop task (reads sensor data, updates UI)
//   - ESP-NOW receive callback (WiFi task context on Core 0)
//   - WiFi event callbacks (system task context)
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Network State
// ============================================================

static bool g_wifiInitialized = false;
static unsigned long g_lastWiFiReconnectAttempt = 0;
static unsigned long g_wifiDisconnectStart = 0;
static unsigned long g_wifiConnectedTime = 0;
static bool g_apModeActive = false;
static bool g_mdnsStarted = false;

// ESP-NOW fridge tracking
static bool g_espnowInitialized = false;
static unsigned long g_lastFridgePacket = 0;
static uint16_t g_lastFridgeSequence = 0;
static bool g_fridgeHeartbeatLost = false;
static bool g_fridgeHeartbeatAlertSent = false;
static bool g_fridgePacketEverReceived = false;

// ============================================================
// CRC-16 Implementation (GH-NET-003, GH-NET-004)
// ============================================================

uint16_t network_crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ ESPNOW_CRC16_POLY;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ============================================================
// ESP-NOW Receive Callback (WiFi Task Context)
// ============================================================

void onESPNOWReceive(const esp_now_recv_info* info, const uint8_t* incomingData, int len) { 
   if (len != sizeof(FridgePacket)) {
  
    Serial.print(F("[ESPNOW] Invalid packet size: "));
    Serial.print(len);
    Serial.print(F(" bytes (expected "));
    Serial.print(sizeof(FridgePacket));
    Serial.println(F(") - dropped"));
    return;
  }

  const FridgePacket* packet = (const FridgePacket*)incomingData;

  uint16_t calculatedCRC = network_crc16(incomingData, sizeof(FridgePacket) - sizeof(uint16_t));
  if (calculatedCRC != packet->crc16) {
    Serial.print(F("[ESPNOW] CRC-16 mismatch! Calc: 0x"));
    Serial.print(calculatedCRC, HEX);
    Serial.print(F(", Received: 0x"));
    Serial.println(packet->crc16, HEX);
    return;
  }

  bool everReceived;
  uint16_t lastSeq;
  portENTER_CRITICAL(&g_stateMux);
  everReceived = g_fridgePacketEverReceived;
  lastSeq = g_lastFridgeSequence;
  portEXIT_CRITICAL(&g_stateMux);

  uint16_t expectedSeq = lastSeq + 1;
  if (everReceived && packet->sequenceNumber != expectedSeq) {
    Serial.print(F("[ESPNOW] Sequence gap - expected "));
    Serial.print(expectedSeq);
    Serial.print(F(", got "));
    Serial.println(packet->sequenceNumber);
  }

  if (packet->temperature < -40.0f || packet->temperature > 85.0f) {
    Serial.print(F("[ESPNOW] Temperature out of valid range: "));
    Serial.println(packet->temperature, 1);
    return;
  }

  portENTER_CRITICAL(&g_stateMux);
  g_lastFridgeSequence = packet->sequenceNumber;
  g_lastFridgePacket = millis();
  g_fridgeHeartbeatLost = false;
  g_fridgeHeartbeatAlertSent = false;
  g_fridgePacketEverReceived = true;
  g_systemState.fridgeTemp = packet->temperature;
  g_systemState.fridgeLastSequence = packet->sequenceNumber;
  g_systemState.fridgeHeartbeatLost = false;
  portEXIT_CRITICAL(&g_stateMux);

  Serial.print(F("[ESPNOW] Fridge packet OK - Seq: "));
  Serial.print(packet->sequenceNumber);
  Serial.print(F(", Temp: "));
  Serial.print(packet->temperature, 2);
  Serial.println(F(" C"));
}

// ============================================================
// FreeRTOS-Safe Data Accessors
// ============================================================

float network_getFridgeTemp() {
  float temp;
  portENTER_CRITICAL(&g_stateMux);
  temp = g_systemState.fridgeTemp;
  portEXIT_CRITICAL(&g_stateMux);
  return temp;
}

bool network_isFridgeHeartbeatLost() {
  bool lost;
  portENTER_CRITICAL(&g_stateMux);
  lost = g_systemState.fridgeHeartbeatLost;
  portEXIT_CRITICAL(&g_stateMux);
  return lost;
}

uint16_t network_getFridgeLastSequence() {
  uint16_t seq;
  portENTER_CRITICAL(&g_stateMux);
  seq = g_systemState.fridgeLastSequence;
  portEXIT_CRITICAL(&g_stateMux);
  return seq;
}

// ============================================================
// WiFi Event Handler
// ============================================================

static void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println(F("[WiFi] Station connected to AP"));
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP: {
      Serial.print(F("[WiFi] Station got IP: "));
      Serial.println(WiFi.localIP());

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.wifiConnected = true;
      g_wifiDisconnectStart = 0;
      g_wifiConnectedTime = millis();

      bool shouldStartMDNS = !g_mdnsStarted;
      if (shouldStartMDNS) {
        g_mdnsStarted = true;
      }
      portEXIT_CRITICAL(&g_stateMux);

      if (shouldStartMDNS) {
        if (MDNS.begin("growhub")) {
          Serial.println(F("[NET] mDNS started: http://growhub.local"));
          MDNS.addService("http", "tcp", 80);
        } else {
          portENTER_CRITICAL(&g_stateMux);
          g_mdnsStarted = false;
          portEXIT_CRITICAL(&g_stateMux);
          Serial.println(F("[NET] mDNS failed to start"));
        }
      }
      break;
    }

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println(F("[WiFi] Station disconnected"));

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.wifiConnected = false;
      if (g_wifiDisconnectStart == 0) {
        g_wifiDisconnectStart = millis();
      }
      g_mdnsStarted = false;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println(F("[WiFi] Access Point started"));
      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = true;
      g_systemState.apModeActive = true;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println(F("[WiFi] Access Point stopped"));
      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = false;
      g_systemState.apModeActive = false;
      g_mdnsStarted = false;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println(F("[WiFi] Client connected to AP"));
      break;

    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println(F("[WiFi] Client disconnected from AP"));
      break;

    default:
      break;
  }
}

// ============================================================
// Network Initialization
// ============================================================

bool network_init() {
  Serial.println(F("[NET] Initializing network subsystem..."));

  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_AP_STA);

  IPAddress staticIP(STATIC_IP_OCTET_1, STATIC_IP_OCTET_2, STATIC_IP_OCTET_3, STATIC_IP_OCTET_4);
  IPAddress gateway(GATEWAY_OCTET_1, GATEWAY_OCTET_2, GATEWAY_OCTET_3, GATEWAY_OCTET_4);
  IPAddress subnet(255, 255, 254, 0);
  IPAddress dns1(8, 8, 8, 8);
  IPAddress dns2(8, 8, 4, 4);

  if (WiFi.config(staticIP, gateway, subnet, dns1, dns2)) {
    Serial.print(F("[NET] Static IP configured: "));
    Serial.println(staticIP);
  } else {
    Serial.println(F("[NET] WARNING: Static IP configuration failed - using DHCP"));
  }

  uint8_t connectAttempts = 0;
  bool connected = false;

  while (connectAttempts < 3 && !connected) {
    connectAttempts++;
    Serial.print(F("[NET] Connecting to WiFi (attempt "));
    Serial.print(connectAttempts);
    Serial.print(F("/3): "));
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500);
      safety_feedWatchdog();
      Serial.print(F("."));
    }

    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
    } else {
      Serial.println(F(""));
      Serial.println(F("[NET] Attempt failed, retrying..."));
      WiFi.disconnect();
      delay(1000);
    }
  }

  if (connected) {
    Serial.println(F(""));
    Serial.println(F("[NET] Connected to WiFi!"));
    Serial.print(F("[NET] Static IP: http://"));
    Serial.print(staticIP);
    Serial.println(F(""));

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = true;
    g_wifiDisconnectStart = 0;
    g_wifiConnectedTime = millis();

    bool shouldStartMDNS = !g_mdnsStarted;
    if (shouldStartMDNS) {
      g_mdnsStarted = true;
    }
    portEXIT_CRITICAL(&g_stateMux);

    if (shouldStartMDNS) {
      if (MDNS.begin("growhub")) {
        Serial.println(F("[NET] mDNS started: http://growhub.local"));
        MDNS.addService("http", "tcp", 80);
      } else {
        portENTER_CRITICAL(&g_stateMux);
        g_mdnsStarted = false;
        portEXIT_CRITICAL(&g_stateMux);
        Serial.println(F("[NET] mDNS failed to start"));
      }
    }
  } else {
    Serial.println(F(""));
    Serial.println(F("[NET] All connection attempts failed."));

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = false;
    g_wifiDisconnectStart = millis() - WIFI_DISCONNECT_TIMEOUT_MS;
    portEXIT_CRITICAL(&g_stateMux);

    network_checkConnection();
  }

  if (esp_now_init() == ESP_OK) {
    Serial.println(F("[ESPNOW] Initialized successfully - listening for fridge node"));
    esp_now_register_recv_cb(onESPNOWReceive);
    g_espnowInitialized = true;
    portENTER_CRITICAL(&g_stateMux);
    g_lastFridgePacket = millis();
    portEXIT_CRITICAL(&g_stateMux);
  } else {
    Serial.println(F("[ESPNOW] Initialization FAILED - fridge monitoring unavailable"));
    g_espnowInitialized = false;
  }

  g_wifiInitialized = true;
  return g_wifiInitialized;
}

// ============================================================
// Connection Monitoring
// ============================================================

void network_checkConnection() {
  unsigned long now = millis();

  portENTER_CRITICAL(&g_stateMux);
  bool apActive = g_apModeActive;
  unsigned long connectedTime = g_wifiConnectedTime;
  portEXIT_CRITICAL(&g_stateMux);

  if (WiFi.status() == WL_CONNECTED) {
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = true;
    g_wifiDisconnectStart = 0;
    portEXIT_CRITICAL(&g_stateMux);

    if (apActive && connectedTime > 0 && (now - connectedTime > 30000)) {
      WiFi.softAPdisconnect(true);

      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = false;
      g_systemState.apModeActive = false;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.println(F("[NET] AP mode disabled - station connection stable"));
    }
    return;
  }

  portENTER_CRITICAL(&g_stateMux);
  g_systemState.wifiConnected = false;
  portEXIT_CRITICAL(&g_stateMux);

  portENTER_CRITICAL(&g_stateMux);
  if (g_wifiDisconnectStart == 0) {
    g_wifiDisconnectStart = now;
  }
  unsigned long disconnectedDuration = now - g_wifiDisconnectStart;
  portEXIT_CRITICAL(&g_stateMux);

  if (disconnectedDuration >= WIFI_DISCONNECT_TIMEOUT_MS && !apActive) {
    Serial.println(F("[NET] WiFi disconnected for 15s - enabling backup AP"));
    Serial.print(F("[NET] AP SSID: "));
    Serial.println(AP_SSID);

    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = true;
      g_systemState.apModeActive = true;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.print(F("[NET] AP IP Address: "));
      Serial.println(WiFi.softAPIP());

      portENTER_CRITICAL(&g_stateMux);
      bool shouldStartMDNS = !g_mdnsStarted;
      if (shouldStartMDNS) {
        g_mdnsStarted = true;
      }
      portEXIT_CRITICAL(&g_stateMux);

      if (shouldStartMDNS) {
        if (MDNS.begin("growhub")) {
          Serial.println(F("[NET] mDNS started: http://growhub.local"));
          MDNS.addService("http", "tcp", 80);
        } else {
          portENTER_CRITICAL(&g_stateMux);
          g_mdnsStarted = false;
          portEXIT_CRITICAL(&g_stateMux);
          Serial.println(F("[NET] mDNS failed to start"));
        }
      }

      network_sendAlert("GrowHub32 - AP Mode Active",
                       "Main Node WiFi lost. Backup AP enabled for direct access.");
    } else {
      Serial.println(F("[NET] Failed to start AP mode"));
    }
  }

  if (now - g_lastWiFiReconnectAttempt >= WIFI_SCAN_INTERVAL_MS) {
    g_lastWiFiReconnectAttempt = now;
    Serial.println(F("[NET] Attempting WiFi reconnection..."));
    WiFi.reconnect();
  }
}

// ============================================================
// Fridge Heartbeat Monitoring
// ============================================================

void network_checkFridgeHeartbeat() {
  if (!g_espnowInitialized) {
    return;
  }

  unsigned long now = millis();

  portENTER_CRITICAL(&g_stateMux);
  bool everReceived = g_fridgePacketEverReceived;
  unsigned long lastPacket = g_lastFridgePacket;
  bool heartbeatLost = g_fridgeHeartbeatLost;
  bool alertSent = g_fridgeHeartbeatAlertSent;
  portEXIT_CRITICAL(&g_stateMux);

  if (!everReceived) {
    return;
  }

  unsigned long elapsedSinceLastPacket = now - lastPacket;

  if (elapsedSinceLastPacket >= ESPNOW_FAIL_TIMEOUT_MS) {
    if (!heartbeatLost) {
      heartbeatLost = true;

      Serial.println(F("[ESPNOW] Fridge node heartbeat LOST! (>60s no data)"));

      if (!alertSent) {
        alertSent = true;
        network_sendAlert("Fridge Node Offline",
                         "No valid data received from fridge node for 60+ seconds.");
      }
    }
  } else {
    if (heartbeatLost) {
      Serial.println(F("[ESPNOW] Fridge node heartbeat RESTORED"));
    }
    heartbeatLost = false;
    alertSent = false;
  }

  portENTER_CRITICAL(&g_stateMux);
  g_fridgeHeartbeatLost = heartbeatLost;
  g_fridgeHeartbeatAlertSent = alertSent;
  g_systemState.fridgeHeartbeatLost = heartbeatLost;
  portEXIT_CRITICAL(&g_stateMux);
}

// ============================================================
// Push Notifications (GH-NET-005)
// ============================================================

void network_sendAlert(const char* title, const char* message) {
  unsigned long now = millis();

  bool rateLimited = false;
  portENTER_CRITICAL(&g_stateMux);
  if (now - g_systemState.lastNTFYAlert < NTFY_MIN_INTERVAL_MS) {
    rateLimited = true;
  }
  portEXIT_CRITICAL(&g_stateMux);

  if (rateLimited) {
    Serial.print(F("[NTFY] Rate limited - skipping alert: "));
    Serial.println(title);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[NTFY] No WiFi connection - alert not sent:"));
    Serial.print(F("[NTFY]   Title: "));
    Serial.println(title);
    Serial.print(F("[NTFY]   Message: "));
    Serial.println(message);
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://" + String(NTFY_ENDPOINT);

  if (!http.begin(client, url)) {
    Serial.print(F("[NTFY] Failed to begin HTTPS connection: "));
    Serial.println(title);
    return;
  }

  http.addHeader("Title", title);
  http.addHeader("Priority", "high");
  http.addHeader("Tags", "warning,rotating_light");
  http.addHeader("Content-Type", "text/plain");

  int httpResponseCode = http.POST(message);

  if (httpResponseCode > 0) {
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.lastNTFYAlert = now;
    portEXIT_CRITICAL(&g_stateMux);

    Serial.print(F("[NTFY] Alert sent: "));
    Serial.print(title);
    Serial.print(F(" (HTTP "));
    Serial.print(httpResponseCode);
    Serial.println(F(")"));
  } else {
    Serial.print(F("[NTFY] Alert FAILED: "));
    Serial.print(title);
    Serial.print(F(" (Error: "));
    Serial.print(httpResponseCode);
    Serial.print(F(" - "));
    Serial.print(http.errorToString(httpResponseCode));
    Serial.println(F(")"));
  }

  http.end();
}

// ============================================================
// Status Queries
// ============================================================

bool network_isWiFiConnected() {
  bool connected;
  portENTER_CRITICAL(&g_stateMux);
  connected = g_systemState.wifiConnected;
  portEXIT_CRITICAL(&g_stateMux);
  return connected;
}

bool network_isAPMode() {
  bool apMode;
  portENTER_CRITICAL(&g_stateMux);
  apMode = g_systemState.apModeActive;
  portEXIT_CRITICAL(&g_stateMux);
  return apMode;
}

String network_getIPAddress() {
  if (network_isWiFiConnected()) {
    return WiFi.localIP().toString();
  }
  if (network_isAPMode()) {
    return WiFi.softAPIP().toString();
  }
  return "No Connection";
}
/*
   network.cpp
   GrowHub32 - Wireless Networking & Alert Subsystem Implementation
   Version: 1.3.0
   Revision: Added system_state.h include. Centralized network credentials.
             Mutex-protected lastNTFYAlert rate-limit check.
             Wraparound-aware sequence number comparison.
             AP stability timer uses g_wifiConnectedTime.
             Mutex-protected heartbeat reads in network_checkFridgeHeartbeat.
             mDNS guarded from repeated begin() calls.
             Immediate AP fallback on boot failure.
             Mutex-protected g_wifiDisconnectStart in event callback.
             Distinguish "never received" from "heartbeat lost".
             lastNTFYAlert updated only after successful HTTP send.
             Mutex-protected reads in network_checkConnection.
             Mutex-protected g_mdnsStarted access.
             Fixed switch scoping bug in onWiFiEvent GOT_IP case.
             Fixed mDNS race condition — check+claim inside single critical section.
             Failed mDNS start reverts g_mdnsStarted flag.
             Added MDNS.addService in AP fallback block for consistency.
             Mutex-protected g_wifiDisconnectStart writes in network_checkConnection.

   WiFi credentials are configured below per GH-NET-001.
   Alert endpoint is configured below per GH-NET-005.
   ESP-NOW listens for Remote Fridge Node packets.
*/

#include "network.h"
#include "sensors.h"
#include "safety.h"
#include "system_state.h"
#include <ESPmDNS.h>

// ============================================================
// Network Credentials (private — do not commit to version control)
// ============================================================
#define WIFI_SSID                       "DSLAP"
#define WIFI_PASSWORD                   "12345678910"
#define AP_SSID                         "GrowHub32_AP"
#define AP_PASSWORD                     "growhub123"

// Static IP configuration
#define STATIC_IP_OCTET_1               10
#define STATIC_IP_OCTET_2               0
#define STATIC_IP_OCTET_3               0
#define STATIC_IP_OCTET_4               20
#define GATEWAY_OCTET_1                 10
#define GATEWAY_OCTET_2                 0
#define GATEWAY_OCTET_3                 0
#define GATEWAY_OCTET_4                 1

// ntfy.sh push notification endpoint
#define NTFY_ENDPOINT                   "ntfy.sh/growhub32_alerts"

// ============================================================
// FreeRTOS Concurrency Guard
// ============================================================

// Protects g_systemState access between:
//   - Main Arduino loop task (reads sensor data, updates UI)
//   - ESP-NOW receive callback (WiFi task context on Core 0)
//   - WiFi event callbacks (system task context)
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Network State
// ============================================================

static bool g_wifiInitialized = false;
static unsigned long g_lastWiFiReconnectAttempt = 0;
static unsigned long g_wifiDisconnectStart = 0;
static unsigned long g_wifiConnectedTime = 0;
static bool g_apModeActive = false;
static bool g_mdnsStarted = false;

// ESP-NOW fridge tracking
static bool g_espnowInitialized = false;
static unsigned long g_lastFridgePacket = 0;
static uint16_t g_lastFridgeSequence = 0;
static bool g_fridgeHeartbeatLost = false;
static bool g_fridgeHeartbeatAlertSent = false;
static bool g_fridgePacketEverReceived = false;

// ============================================================
// CRC-16 Implementation (GH-NET-003, GH-NET-004)
// ============================================================

uint16_t network_crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ ESPNOW_CRC16_POLY;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ============================================================
// ESP-NOW Receive Callback (WiFi Task Context)
// ============================================================

void onESPNOWReceive(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (len != sizeof(FridgePacket)) {
    Serial.print(F("[ESPNOW] Invalid packet size: "));
    Serial.print(len);
    Serial.print(F(" bytes (expected "));
    Serial.print(sizeof(FridgePacket));
    Serial.println(F(") - dropped"));
    return;
  }

  const FridgePacket* packet = (const FridgePacket*)incomingData;

  uint16_t calculatedCRC = network_crc16(incomingData, sizeof(FridgePacket) - sizeof(uint16_t));
  if (calculatedCRC != packet->crc16) {
    Serial.print(F("[ESPNOW] CRC-16 mismatch! Calc: 0x"));
    Serial.print(calculatedCRC, HEX);
    Serial.print(F(", Received: 0x"));
    Serial.println(packet->crc16, HEX);
    return;
  }

  bool everReceived;
  uint16_t lastSeq;
  portENTER_CRITICAL(&g_stateMux);
  everReceived = g_fridgePacketEverReceived;
  lastSeq = g_lastFridgeSequence;
  portEXIT_CRITICAL(&g_stateMux);

  uint16_t expectedSeq = lastSeq + 1;
  if (everReceived && packet->sequenceNumber != expectedSeq) {
    Serial.print(F("[ESPNOW] Sequence gap - expected "));
    Serial.print(expectedSeq);
    Serial.print(F(", got "));
    Serial.println(packet->sequenceNumber);
  }

  if (packet->temperature < -40.0f || packet->temperature > 85.0f) {
    Serial.print(F("[ESPNOW] Temperature out of valid range: "));
    Serial.println(packet->temperature, 1);
    return;
  }

  portENTER_CRITICAL(&g_stateMux);
  g_lastFridgeSequence = packet->sequenceNumber;
  g_lastFridgePacket = millis();
  g_fridgeHeartbeatLost = false;
  g_fridgeHeartbeatAlertSent = false;
  g_fridgePacketEverReceived = true;
  g_systemState.fridgeTemp = packet->temperature;
  g_systemState.fridgeLastSequence = packet->sequenceNumber;
  g_systemState.fridgeHeartbeatLost = false;
  portEXIT_CRITICAL(&g_stateMux);

  Serial.print(F("[ESPNOW] Fridge packet OK - Seq: "));
  Serial.print(packet->sequenceNumber);
  Serial.print(F(", Temp: "));
  Serial.print(packet->temperature, 2);
  Serial.println(F(" C"));
}

// ============================================================
// FreeRTOS-Safe Data Accessors
// ============================================================

float network_getFridgeTemp() {
  float temp;
  portENTER_CRITICAL(&g_stateMux);
  temp = g_systemState.fridgeTemp;
  portEXIT_CRITICAL(&g_stateMux);
  return temp;
}

bool network_isFridgeHeartbeatLost() {
  bool lost;
  portENTER_CRITICAL(&g_stateMux);
  lost = g_systemState.fridgeHeartbeatLost;
  portEXIT_CRITICAL(&g_stateMux);
  return lost;
}

uint16_t network_getFridgeLastSequence() {
  uint16_t seq;
  portENTER_CRITICAL(&g_stateMux);
  seq = g_systemState.fridgeLastSequence;
  portEXIT_CRITICAL(&g_stateMux);
  return seq;
}

// ============================================================
// WiFi Event Handler
// ============================================================

static void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println(F("[WiFi] Station connected to AP"));
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP: {
      Serial.print(F("[WiFi] Station got IP: "));
      Serial.println(WiFi.localIP());

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.wifiConnected = true;
      g_wifiDisconnectStart = 0;
      g_wifiConnectedTime = millis();

      bool shouldStartMDNS = !g_mdnsStarted;
      if (shouldStartMDNS) {
        g_mdnsStarted = true;
      }
      portEXIT_CRITICAL(&g_stateMux);

      if (shouldStartMDNS) {
        if (MDNS.begin("growhub")) {
          Serial.println(F("[NET] mDNS started: http://growhub.local"));
          MDNS.addService("http", "tcp", 80);
        } else {
          portENTER_CRITICAL(&g_stateMux);
          g_mdnsStarted = false;
          portEXIT_CRITICAL(&g_stateMux);
          Serial.println(F("[NET] mDNS failed to start"));
        }
      }
      break;
    }

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println(F("[WiFi] Station disconnected"));

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.wifiConnected = false;
      if (g_wifiDisconnectStart == 0) {
        g_wifiDisconnectStart = millis();
      }
      g_mdnsStarted = false;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println(F("[WiFi] Access Point started"));
      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = true;
      g_systemState.apModeActive = true;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println(F("[WiFi] Access Point stopped"));
      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = false;
      g_systemState.apModeActive = false;
      g_mdnsStarted = false;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println(F("[WiFi] Client connected to AP"));
      break;

    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println(F("[WiFi] Client disconnected from AP"));
      break;

    default:
      break;
  }
}

// ============================================================
// Network Initialization
// ============================================================

bool network_init() {
  Serial.println(F("[NET] Initializing network subsystem..."));

  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_AP_STA);

  IPAddress staticIP(STATIC_IP_OCTET_1, STATIC_IP_OCTET_2, STATIC_IP_OCTET_3, STATIC_IP_OCTET_4);
  IPAddress gateway(GATEWAY_OCTET_1, GATEWAY_OCTET_2, GATEWAY_OCTET_3, GATEWAY_OCTET_4);
  IPAddress subnet(255, 255, 254, 0);
  IPAddress dns1(8, 8, 8, 8);
  IPAddress dns2(8, 8, 4, 4);

  if (WiFi.config(staticIP, gateway, subnet, dns1, dns2)) {
    Serial.print(F("[NET] Static IP configured: "));
    Serial.println(staticIP);
  } else {
    Serial.println(F("[NET] WARNING: Static IP configuration failed - using DHCP"));
  }

  uint8_t connectAttempts = 0;
  bool connected = false;

  while (connectAttempts < 3 && !connected) {
    connectAttempts++;
    Serial.print(F("[NET] Connecting to WiFi (attempt "));
    Serial.print(connectAttempts);
    Serial.print(F("/3): "));
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500);
      safety_feedWatchdog();
      Serial.print(F("."));
    }

    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
    } else {
      Serial.println(F(""));
      Serial.println(F("[NET] Attempt failed, retrying..."));
      WiFi.disconnect();
      delay(1000);
    }
  }

  if (connected) {
    Serial.println(F(""));
    Serial.println(F("[NET] Connected to WiFi!"));
    Serial.print(F("[NET] Static IP: http://"));
    Serial.print(staticIP);
    Serial.println(F(""));

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = true;
    g_wifiDisconnectStart = 0;
    g_wifiConnectedTime = millis();

    bool shouldStartMDNS = !g_mdnsStarted;
    if (shouldStartMDNS) {
      g_mdnsStarted = true;
    }
    portEXIT_CRITICAL(&g_stateMux);

    if (shouldStartMDNS) {
      if (MDNS.begin("growhub")) {
        Serial.println(F("[NET] mDNS started: http://growhub.local"));
        MDNS.addService("http", "tcp", 80);
      } else {
        portENTER_CRITICAL(&g_stateMux);
        g_mdnsStarted = false;
        portEXIT_CRITICAL(&g_stateMux);
        Serial.println(F("[NET] mDNS failed to start"));
      }
    }
  } else {
    Serial.println(F(""));
    Serial.println(F("[NET] All connection attempts failed."));

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = false;
    g_wifiDisconnectStart = millis() - WIFI_DISCONNECT_TIMEOUT_MS;
    portEXIT_CRITICAL(&g_stateMux);

    network_checkConnection();
  }

  if (esp_now_init() == ESP_OK) {
    Serial.println(F("[ESPNOW] Initialized successfully - listening for fridge node"));
    esp_now_register_recv_cb(onESPNOWReceive);
    g_espnowInitialized = true;
    portENTER_CRITICAL(&g_stateMux);
    g_lastFridgePacket = millis();
    portEXIT_CRITICAL(&g_stateMux);
  } else {
    Serial.println(F("[ESPNOW] Initialization FAILED - fridge monitoring unavailable"));
    g_espnowInitialized = false;
  }

  g_wifiInitialized = true;
  return g_wifiInitialized;
}

// ============================================================
// Connection Monitoring
// ============================================================

void network_checkConnection() {
  unsigned long now = millis();

  portENTER_CRITICAL(&g_stateMux);
  bool apActive = g_apModeActive;
  unsigned long connectedTime = g_wifiConnectedTime;
  portEXIT_CRITICAL(&g_stateMux);

  if (WiFi.status() == WL_CONNECTED) {
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = true;
    g_wifiDisconnectStart = 0;
    portEXIT_CRITICAL(&g_stateMux);

    if (apActive && connectedTime > 0 && (now - connectedTime > 30000)) {
      WiFi.softAPdisconnect(true);

      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = false;
      g_systemState.apModeActive = false;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.println(F("[NET] AP mode disabled - station connection stable"));
    }
    return;
  }

  portENTER_CRITICAL(&g_stateMux);
  g_systemState.wifiConnected = false;
  portEXIT_CRITICAL(&g_stateMux);

  portENTER_CRITICAL(&g_stateMux);
  if (g_wifiDisconnectStart == 0) {
    g_wifiDisconnectStart = now;
  }
  unsigned long disconnectedDuration = now - g_wifiDisconnectStart;
  portEXIT_CRITICAL(&g_stateMux);

  if (disconnectedDuration >= WIFI_DISCONNECT_TIMEOUT_MS && !apActive) {
    Serial.println(F("[NET] WiFi disconnected for 15s - enabling backup AP"));
    Serial.print(F("[NET] AP SSID: "));
    Serial.println(AP_SSID);

    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
      portENTER_CRITICAL(&g_stateMux);
      g_apModeActive = true;
      g_systemState.apModeActive = true;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.print(F("[NET] AP IP Address: "));
      Serial.println(WiFi.softAPIP());

      portENTER_CRITICAL(&g_stateMux);
      bool shouldStartMDNS = !g_mdnsStarted;
      if (shouldStartMDNS) {
        g_mdnsStarted = true;
      }
      portEXIT_CRITICAL(&g_stateMux);

      if (shouldStartMDNS) {
        if (MDNS.begin("growhub")) {
          Serial.println(F("[NET] mDNS started: http://growhub.local"));
          MDNS.addService("http", "tcp", 80);
        } else {
          portENTER_CRITICAL(&g_stateMux);
          g_mdnsStarted = false;
          portEXIT_CRITICAL(&g_stateMux);
          Serial.println(F("[NET] mDNS failed to start"));
        }
      }

      network_sendAlert("GrowHub32 - AP Mode Active",
                       "Main Node WiFi lost. Backup AP enabled for direct access.");
    } else {
      Serial.println(F("[NET] Failed to start AP mode"));
    }
  }

  if (now - g_lastWiFiReconnectAttempt >= WIFI_SCAN_INTERVAL_MS) {
    g_lastWiFiReconnectAttempt = now;
    Serial.println(F("[NET] Attempting WiFi reconnection..."));
    WiFi.reconnect();
  }
}

// ============================================================
// Fridge Heartbeat Monitoring
// ============================================================

void network_checkFridgeHeartbeat() {
  if (!g_espnowInitialized) {
    return;
  }

  unsigned long now = millis();

  portENTER_CRITICAL(&g_stateMux);
  bool everReceived = g_fridgePacketEverReceived;
  unsigned long lastPacket = g_lastFridgePacket;
  bool heartbeatLost = g_fridgeHeartbeatLost;
  bool alertSent = g_fridgeHeartbeatAlertSent;
  portEXIT_CRITICAL(&g_stateMux);

  if (!everReceived) {
    return;
  }

  unsigned long elapsedSinceLastPacket = now - lastPacket;

  if (elapsedSinceLastPacket >= ESPNOW_FAIL_TIMEOUT_MS) {
    if (!heartbeatLost) {
      heartbeatLost = true;

      Serial.println(F("[ESPNOW] Fridge node heartbeat LOST! (>60s no data)"));

      if (!alertSent) {
        alertSent = true;
        network_sendAlert("Fridge Node Offline",
                         "No valid data received from fridge node for 60+ seconds.");
      }
    }
  } else {
    if (heartbeatLost) {
      Serial.println(F("[ESPNOW] Fridge node heartbeat RESTORED"));
    }
    heartbeatLost = false;
    alertSent = false;
  }

  portENTER_CRITICAL(&g_stateMux);
  g_fridgeHeartbeatLost = heartbeatLost;
  g_fridgeHeartbeatAlertSent = alertSent;
  g_systemState.fridgeHeartbeatLost = heartbeatLost;
  portEXIT_CRITICAL(&g_stateMux);
}

// ============================================================
// Push Notifications (GH-NET-005)
// ============================================================

void network_sendAlert(const char* title, const char* message) {
  unsigned long now = millis();

  bool rateLimited = false;
  portENTER_CRITICAL(&g_stateMux);
  if (now - g_systemState.lastNTFYAlert < NTFY_MIN_INTERVAL_MS) {
    rateLimited = true;
  }
  portEXIT_CRITICAL(&g_stateMux);

  if (rateLimited) {
    Serial.print(F("[NTFY] Rate limited - skipping alert: "));
    Serial.println(title);
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[NTFY] No WiFi connection - alert not sent:"));
    Serial.print(F("[NTFY]   Title: "));
    Serial.println(title);
    Serial.print(F("[NTFY]   Message: "));
    Serial.println(message);
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://" + String(NTFY_ENDPOINT);

  if (!http.begin(client, url)) {
    Serial.print(F("[NTFY] Failed to begin HTTPS connection: "));
    Serial.println(title);
    return;
  }

  http.addHeader("Title", title);
  http.addHeader("Priority", "high");
  http.addHeader("Tags", "warning,rotating_light");
  http.addHeader("Content-Type", "text/plain");

  int httpResponseCode = http.POST(message);

  if (httpResponseCode > 0) {
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.lastNTFYAlert = now;
    portEXIT_CRITICAL(&g_stateMux);

    Serial.print(F("[NTFY] Alert sent: "));
    Serial.print(title);
    Serial.print(F(" (HTTP "));
    Serial.print(httpResponseCode);
    Serial.println(F(")"));
  } else {
    Serial.print(F("[NTFY] Alert FAILED: "));
    Serial.print(title);
    Serial.print(F(" (Error: "));
    Serial.print(httpResponseCode);
    Serial.print(F(" - "));
    Serial.print(http.errorToString(httpResponseCode));
    Serial.println(F(")"));
  }

  http.end();
}

// ============================================================
// Status Queries
// ============================================================

bool network_isWiFiConnected() {
  bool connected;
  portENTER_CRITICAL(&g_stateMux);
  connected = g_systemState.wifiConnected;
  portEXIT_CRITICAL(&g_stateMux);
  return connected;
}

bool network_isAPMode() {
  bool apMode;
  portENTER_CRITICAL(&g_stateMux);
  apMode = g_systemState.apModeActive;
  portEXIT_CRITICAL(&g_stateMux);
  return apMode;
}

String network_getIPAddress() {
  if (network_isWiFiConnected()) {
    return WiFi.localIP().toString();
  }
  if (network_isAPMode()) {
    return WiFi.softAPIP().toString();
  }
  return "No Connection";
}
