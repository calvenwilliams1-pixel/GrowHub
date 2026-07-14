/*
   network.cpp
   GrowHub32 - Wireless Networking & Alert Subsystem Implementation
   Version: 1.2.2
   Revision: Added FreeRTOS concurrency mutex for ESP-NOW callback safety.
             Fixed ntfy.sh HTTPS with explicit WiFiClientSecure.
             Added watchdog feed during initial WiFi connection loop.
             Added static IP configuration.
             Added mDNS support (growhub.local).
             Reduced WiFi reconnect interval to 30 seconds.
             Added 3-attempt initial connection with 10s timeout per attempt.

   WiFi credentials are hardcoded per GH-NET-001.
   Alert endpoint is hardcoded per GH-NET-005.
   ESP-NOW listens for Remote Fridge Node packets.
*/

#include "network.h"
#include "sensors.h"
#include "safety.h"
#include <ESPmDNS.h>

// ============================================
// FreeRTOS Concurrency Guard
// ============================================

// Protects g_systemState access between:
//   - Main Arduino loop task (reads sensor data, updates UI)
//   - ESP-NOW receive callback (WiFi task context on Core 0)
//   - WiFi event callbacks (system task context)
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================
// Network State
// ============================================

static bool g_wifiInitialized = false;
static unsigned long g_lastWiFiReconnectAttempt = 0;
static unsigned long g_wifiDisconnectStart = 0;
static bool g_apModeActive = false;

// ESP-NOW fridge tracking
static bool g_espnowInitialized = false;
static unsigned long g_lastFridgePacket = 0;
static uint16_t g_lastFridgeSequence = 0;
static bool g_fridgeHeartbeatLost = false;
static bool g_fridgeHeartbeatAlertSent = false;

// ============================================
// CRC-16 Implementation (GH-NET-003, GH-NET-004)
// ============================================

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

// ============================================
// ESP-NOW Receive Callback (WiFi Task Context)
// ============================================

void onESPNOWReceive(const uint8_t* mac, const uint8_t* incomingData, int len) {
  // GH-NET-003: Validate packet structure
  if (len != sizeof(FridgePacket)) {
    Serial.print(F("[ESPNOW] Invalid packet size: "));
    Serial.print(len);
    Serial.print(F(" bytes (expected "));
    Serial.print(sizeof(FridgePacket));
    Serial.println(F(") - dropped"));
    return;
  }

  const FridgePacket* packet = (const FridgePacket*)incomingData;

  // GH-NET-004: Validate CRC-16 checksum
  // CRC is computed over all bytes except the CRC field itself
  uint16_t calculatedCRC = network_crc16(incomingData, sizeof(FridgePacket) - sizeof(uint16_t));
  if (calculatedCRC != packet->crc16) {
    Serial.print(F("[ESPNOW] CRC-16 mismatch! Calc: 0x"));
    Serial.print(calculatedCRC, HEX);
    Serial.print(F(", Received: 0x"));
    Serial.println(packet->crc16, HEX);
    return; // Drop corrupted packet silently
  }

  // Validate sequence number ordering
  if (packet->sequenceNumber <= g_lastFridgeSequence && g_lastFridgeSequence > 0) {
    // Non-incrementing sequence - possible duplicate or out-of-order packet
    // Accept the data but log a warning (could indicate node reset or RF issue)
    Serial.print(F("[ESPNOW] Non-incrementing sequence: "));
    Serial.print(packet->sequenceNumber);
    Serial.print(F(" (last valid: "));
    Serial.print(g_lastFridgeSequence);
    Serial.println(F(") - possible duplicate"));
  }

  // Validate temperature range (-40C to +85C is DS18B20 range)
  if (packet->temperature < -40.0f || packet->temperature > 85.0f) {
    Serial.print(F("[ESPNOW] Temperature out of valid range: "));
    Serial.println(packet->temperature, 1);
    return; // Drop invalid data
  }

  // ============================================
  // FREE RTOS CRITICAL SECTION
  // This callback runs in WiFi task context.
  // g_systemState is also read by the main loop task.
  // Mutex prevents torn reads and cache coherency issues
  // on dual-core ESP32.
  // ============================================
  portENTER_CRITICAL(&g_stateMux);

  g_lastFridgeSequence = packet->sequenceNumber;
  g_lastFridgePacket = millis();
  g_fridgeHeartbeatLost = false;
  g_fridgeHeartbeatAlertSent = false;
  g_systemState.fridgeTemp = packet->temperature;
  g_systemState.fridgeLastSequence = packet->sequenceNumber;
  g_systemState.fridgeHeartbeatLost = false;

  portEXIT_CRITICAL(&g_stateMux);

  // Log outside critical section to avoid blocking the mutex
  Serial.print(F("[ESPNOW] Fridge packet OK - Seq: "));
  Serial.print(packet->sequenceNumber);
  Serial.print(F(", Temp: "));
  Serial.print(packet->temperature, 2);
  Serial.println(F(" C"));
}

// ============================================
// FreeRTOS-Safe Data Accessors
// ============================================

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

// ============================================
// WiFi Event Handler
// ============================================

static void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println(F("[WiFi] Station connected to AP"));
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print(F("[WiFi] Station got IP: "));
      Serial.println(WiFi.localIP());

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.wifiConnected = true;
      portEXIT_CRITICAL(&g_stateMux);

      g_wifiDisconnectStart = 0;

      // Start mDNS responder
      if (MDNS.begin("growhub")) {
        Serial.println(F("[NET] mDNS started: http://growhub.local"));
        MDNS.addService("http", "tcp", 80);
      } else {
        Serial.println(F("[NET] mDNS failed to start"));
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println(F("[WiFi] Station disconnected"));

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.wifiConnected = false;
      portEXIT_CRITICAL(&g_stateMux);

      if (g_wifiDisconnectStart == 0) {
        g_wifiDisconnectStart = millis();
      }
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println(F("[WiFi] Access Point started"));
      g_apModeActive = true;

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.apModeActive = true;
      portEXIT_CRITICAL(&g_stateMux);
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println(F("[WiFi] Access Point stopped"));
      g_apModeActive = false;

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.apModeActive = false;
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

// ============================================
// Network Initialization
// ============================================

bool network_init() {
  Serial.println(F("[NET] Initializing network subsystem..."));

  // Register WiFi event handler
  WiFi.onEvent(onWiFiEvent);

  // Set WiFi mode to both Station and AP (AP as backup - GH-NET-002)
  WiFi.mode(WIFI_AP_STA);

  // --- Configure Static IP ---
  IPAddress staticIP(STATIC_IP_OCTET_1, STATIC_IP_OCTET_2, STATIC_IP_OCTET_3, STATIC_IP_OCTET_4);
  IPAddress gateway(GATEWAY_OCTET_1, GATEWAY_OCTET_2, GATEWAY_OCTET_3, GATEWAY_OCTET_4);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns1(8, 8, 8, 8);
  IPAddress dns2(8, 8, 4, 4);

  if (WiFi.config(staticIP, gateway, subnet, dns1, dns2)) {
    Serial.print(F("[NET] Static IP configured: "));
    Serial.println(staticIP);
  } else {
    Serial.println(F("[NET] WARNING: Static IP configuration failed - using DHCP"));
  }

  // --- Connect to primary router with retries ---
  uint8_t connectAttempts = 0;
  bool connected = false;

  while (connectAttempts < 3 && !connected) {
    connectAttempts++;
    Serial.print(F("[NET] Connecting to WiFi (attempt "));
    Serial.print(connectAttempts);
    Serial.print(F("/3): "));
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait up to 10 seconds per attempt
    // GH-SAFE-006: Feed watchdog during this blocking loop
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
    portEXIT_CRITICAL(&g_stateMux);

    g_wifiDisconnectStart = 0;

    // Start mDNS responder
    if (MDNS.begin("growhub")) {
      Serial.println(F("[NET] mDNS started: http://growhub.local"));
      MDNS.addService("http", "tcp", 80);
    } else {
      Serial.println(F("[NET] mDNS failed to start"));
    }
  } else {
    Serial.println(F(""));
    Serial.println(F("[NET] All connection attempts failed."));

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = false;
    portEXIT_CRITICAL(&g_stateMux);

    // GH-NET-002: Start backup AP immediately if no connection
    network_checkConnection();
  }

  // --- Initialize ESP-NOW for Fridge Node ---
  if (esp_now_init() == ESP_OK) {
    Serial.println(F("[ESPNOW] Initialized successfully - listening for fridge node"));
    esp_now_register_recv_cb(onESPNOWReceive);
    g_espnowInitialized = true;
    g_lastFridgePacket = millis();
  } else {
    Serial.println(F("[ESPNOW] Initialization FAILED - fridge monitoring unavailable"));
    g_espnowInitialized = false;
  }

  g_wifiInitialized = true;
  return g_wifiInitialized;
}

// ============================================
// Connection Monitoring
// ============================================

void network_checkConnection() {
  // GH-NET-002: Monitor WiFi connection and manage AP fallback
  unsigned long now = millis();

  // Check if station is connected
  if (WiFi.status() == WL_CONNECTED) {
    portENTER_CRITICAL(&g_stateMux);
    g_systemState.wifiConnected = true;
    portEXIT_CRITICAL(&g_stateMux);

    g_wifiDisconnectStart = 0;

    // If AP was active and station is stable for 30 seconds, disable AP
    if (g_apModeActive && now - g_lastWiFiReconnectAttempt > 30000) {
      WiFi.softAPdisconnect(true);
      g_apModeActive = false;

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.apModeActive = false;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.println(F("[NET] AP mode disabled - station connection stable"));
    }
    return;
  }

  // Station is disconnected
  portENTER_CRITICAL(&g_stateMux);
  g_systemState.wifiConnected = false;
  portEXIT_CRITICAL(&g_stateMux);

  // Track disconnection duration
  if (g_wifiDisconnectStart == 0) {
    g_wifiDisconnectStart = now;
  }

  unsigned long disconnectedDuration = now - g_wifiDisconnectStart;

  // GH-NET-002: After 15 seconds, enable backup AP
  if (disconnectedDuration >= WIFI_DISCONNECT_TIMEOUT_MS && !g_apModeActive) {
    Serial.println(F("[NET] WiFi disconnected for 15s - enabling backup AP"));
    Serial.print(F("[NET] AP SSID: "));
    Serial.println(AP_SSID);

    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
      g_apModeActive = true;

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.apModeActive = true;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.print(F("[NET] AP IP Address: "));
      Serial.println(WiFi.softAPIP());

      // Start mDNS in AP mode
      MDNS.begin("growhub");
      Serial.println(F("[NET] mDNS started: http://growhub.local"));

      network_sendAlert("GrowHub32 - AP Mode Active",
                       "Main Node WiFi lost. Backup AP enabled for direct access.");
    } else {
      Serial.println(F("[NET] Failed to start AP mode"));
    }
  }

  // GH-NET-002: Attempt reconnection every 30 seconds
  if (now - g_lastWiFiReconnectAttempt >= WIFI_SCAN_INTERVAL_MS) {
    g_lastWiFiReconnectAttempt = now;
    Serial.println(F("[NET] Attempting WiFi reconnection..."));
    WiFi.reconnect();
  }
}

// ============================================
// Fridge Heartbeat Monitoring
// ============================================

void network_checkFridgeHeartbeat() {
  // GH-NET-004: Monitor fridge node heartbeat
  if (!g_espnowInitialized) {
    return;
  }

  unsigned long now = millis();
  unsigned long elapsedSinceLastPacket = now - g_lastFridgePacket;

  // Check if heartbeat is lost (>60 seconds since last valid packet)
  if (elapsedSinceLastPacket >= ESPNOW_FAIL_TIMEOUT_MS) {
    if (!g_fridgeHeartbeatLost) {
      g_fridgeHeartbeatLost = true;

      portENTER_CRITICAL(&g_stateMux);
      g_systemState.fridgeHeartbeatLost = true;
      portEXIT_CRITICAL(&g_stateMux);

      Serial.println(F("[ESPNOW] Fridge node heartbeat LOST! (>60s no data)"));

      if (!g_fridgeHeartbeatAlertSent) {
        g_fridgeHeartbeatAlertSent = true;
        network_sendAlert("Fridge Node Offline",
                         "No valid data received from fridge node for 60+ seconds.");
      }
    }
  } else {
    if (g_fridgeHeartbeatLost) {
      Serial.println(F("[ESPNOW] Fridge node heartbeat RESTORED"));
    }
    g_fridgeHeartbeatLost = false;

    portENTER_CRITICAL(&g_stateMux);
    g_systemState.fridgeHeartbeatLost = false;
    portEXIT_CRITICAL(&g_stateMux);
  }
}

// ============================================
// Push Notifications (GH-NET-005)
// ============================================

void network_sendAlert(const char* title, const char* message) {
  // GH-NET-005: Send push notification via ntfy.sh
  // Rate-limited to one alert per NTFY_MIN_INTERVAL_MS
  // Uses explicit WiFiClientSecure for reliable HTTPS

  unsigned long now = millis();

  if (now - g_systemState.lastNTFYAlert < NTFY_MIN_INTERVAL_MS) {
    Serial.print(F("[NTFY] Rate limited - skipping alert: "));
    Serial.println(title);
    return;
  }

  g_systemState.lastNTFYAlert = now;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[NTFY] No WiFi connection - alert not sent:"));
    Serial.print(F("[NTFY]   Title: "));
    Serial.println(title);
    Serial.print(F("[NTFY]   Message: "));
    Serial.println(message);
    return;
  }

  // Use explicit WiFiClientSecure for reliable HTTPS
  WiFiClientSecure client;
  client.setInsecure();  // Accept ntfy.sh certificate without CA validation
                         // (Acceptable for push notifications on private network)

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

// ============================================
// Status Queries
// ============================================

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
  } else if (g_apModeActive) {
    return WiFi.softAPIP().toString();
  }
  return "No Connection";
}
