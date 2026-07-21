/*
   web_ui.h
   GrowHub32 - Local Web Application Interface
   Version: 1.2.1

   Provides:
   - Mobile-optimized multi-tab web UI (GH-UI-001)
   - Real-time sensor dashboard with fault banners (GH-UI-002)
   - Calibration control panel with countdown (GH-UI-003)
   - Simulation/projection engine display (GH-UI-004)
   - System log and alert display (GH-UI-005)
   - Threshold configuration interface
   - Manual relay override controls
*/

#ifndef WEB_UI_H
#define WEB_UI_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "config.h"

// WebSocket message types
enum WSMessageType {
  WS_SENSOR_UPDATE = 0,
  WS_RELAY_STATE,
  WS_ALERT,
  WS_THRESHOLD_UPDATE,
  WS_CALIBRATION_STATUS,
  WS_SYSTEM_LOG,
  WS_COMMAND
};

// Public API
bool webUI_init();
void webUI_handleClient();
void webUI_pushUpdates();

#endif // WEB_UI_H
