/*
   web_ui.cpp
   GrowHub32 - Local Web Application Interface Implementation
   Version: 1.4.0
   Revision: Added "Mascots" tab with interactive mushroom designer.
             Added background mushroom "stars" — floating, spinning, pulsing
             custom mushroom designs that drift in the background.
             Uses nested DOM layers for transform isolation, box-shadow
             sprite rendering for performance, and localStorage for settings.

   This serves a single-page application from program memory.
   Chart.js is served from SPIFFS for cache efficiency (v1.4).

   The UI connects via WebSocket on port 81 for real-time updates.
   HTTP server runs on port 80.
*/

#include "web_ui.h"
#include "sensors.h"
#include "relay_manager.h"
#include "automation.h"
#include "adaptive.h"
#include "rtc_handler.h"
#include "safety.h"
#include "network.h"
#include "system_state.h"
#include "sd_logger.h"
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <freertos/semphr.h>

// ============================================================
// UPDATE VERSION HERE WHEN BUMPING FIRMWARE
// ============================================================
#define WEB_UI_VERSION "1.4.0"

static WebServer g_server(WEB_SERVER_PORT);
static WebSocketsServer g_webSocket(WEBSOCKET_PORT);
static unsigned long g_lastWSUpdate = 0;

extern portMUX_TYPE g_stateMux;
extern RuntimeCache g_runtimeCache;
extern unsigned long g_compressorWarmupStart;
extern unsigned long g_compressorWarmupDuration;
extern bool g_warmupSelected;

// ============================================================
// Mascots Tab — Profile Storage Constants
// ============================================================
#define MAX_PROFILES 8
#define PROFILES_FILE "/profiles.json"
#define PROFILES_TEMP "/profiles.tmp"
#define PROFILES_BACKUP "/profiles.json.bak"

static SemaphoreHandle_t g_profileMutex = NULL;

// ============================================================
// Forward Declarations
// ============================================================
static void handleRoot();
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
static void sendSensorUpdate();
static void sendSystemStatus();
static void sendConfigUpdate(uint8_t clientNum);
static void sendCalibrationUpdate();

enum class LoadResult { OK, Empty, Recovered, Failed };
static LoadResult loadProfilesJson(JsonDocument& doc, bool& wasRecovered);
static bool saveProfilesJson(const JsonDocument& doc);
static void getProfileNames(JsonArray& names);
static bool validateProfileData(const JsonObject& data);
static void sendProfileResponse(uint8_t num, const char* cmd, const char* status, const char* message);
static void handleProfileList(uint8_t num);
static void handleProfileSave(uint8_t num, const JsonDocument& req);
static void handleProfileLoad(uint8_t num, const JsonDocument& req);
static void handleProfileDelete(uint8_t num, const JsonDocument& req);
static void recoverProfiles();

// ============================================================
// EMBEDDED HTML/CSS/JS (Single Page Application)
// ============================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>GrowHub32 v1.4.0</title>
<style>
  /* ─── Reset & Base ─── */
  *{margin:0;padding:0;box-sizing:border-box;}
  body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:linear-gradient(180deg,#0d1117 0%,#1a1a2e 50%,#111827 100%);color:#c9d1d9;min-height:100vh;display:flex;flex-direction:column;}

  /* ─── Header ─── */
  .header{padding:14px 18px;border-bottom:1px solid #2a2a4a;box-shadow:0 2px 12px rgba(184,74,255,0.04);position:relative;z-index:30;}
  .header h1{font-size:1.3em;color:#58a6ff;}
  .header .status{font-size:0.75em;color:#8b949e;margin-top:3px;}
  .warning-banner{color:#fff;text-align:center;padding:10px 16px;font-weight:600;display:none;border-bottom:1px solid #f85149;position:relative;z-index:30;}
  .warning-banner.active{display:flex;align-items:center;justify-content:center;gap:8px;animation:pulse-danger 2s infinite;}
  @keyframes pulse-danger{0%{background-color:#da3633;}50%{background-color:#8e1519;}100%{background-color:#da3633;}}
  @keyframes pulse-blue{0%{opacity:1;background:#58a6ff;}50%{opacity:0.7;background:#1f6feb;}100%{opacity:1;background:#58a6ff;}}

  /* ─── Tabs ─── */
  .tabs{display:flex;background:rgba(22,27,34,0.95);backdrop-filter:blur(5px);border-bottom:1px solid #2a2a4a;overflow-x:auto;box-shadow:0 2px 12px rgba(184,74,255,0.04);position:relative;z-index:30;}
  .tab{padding:14px 20px;font-size:0.9em;color:#8b949e;border:none;background:none;cursor:pointer;white-space:nowrap;border-bottom:2px solid transparent;transition:all 0.2s;}
  .tab.active{color:#b84aff;border-bottom-color:#b84aff;text-shadow:0 0 20px rgba(184,74,255,0.3);}
  .tab-content{display:none;padding:16px;flex:1;position:relative;z-index:10;}
  .tab-content.active{display:block;}

  /* ─── Cards & Grids ─── */
  .sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:14px;margin-bottom:18px;}
  .sensor-card{background:#161b22;border:1px solid #2a2a4a;border-radius:12px;padding:18px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.5),0 0 16px rgba(184,74,255,0.06);position:relative;z-index:10;}
  .sensor-card .label{font-size:0.7em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;}
  .sensor-card .value{font-size:2.2em;font-weight:700;margin:8px 0;color:#ffffff;line-height:1;font-variant-numeric:tabular-nums;min-height:1.2em;}
  .sensor-card .unit{font-size:0.6em;color:#8b949e;font-weight:500;text-transform:uppercase;letter-spacing:0.5px;}
  .sensor-card .status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;}
  .sensor-card .status-dot.ok{background:#3fb950;box-shadow:0 0 6px rgba(63,185,80,0.4);}
  .sensor-card .status-dot.fault{background:#da3633;box-shadow:0 0 6px rgba(218,54,51,0.4);}

  .relay-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin-bottom:18px;}
  .relay-card{background:#161b22;border:1px solid #2a2a4a;border-radius:12px;padding:14px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.5),0 0 12px rgba(184,74,255,0.06);position:relative;z-index:10;}
  .relay-card.active{border-color:#b84aff;box-shadow:0 0 20px rgba(184,74,255,0.3),0 0 12px rgba(184,74,255,0.15);}
  .relay-card .name{font-size:0.7em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;}
  .relay-card .state{font-size:1.1em;font-weight:bold;margin:6px 0;}
  .relay-card .state.on{color:#3fb950;}
  .relay-card .state.off{color:#8b949e;}
  .relay-card .locked{color:#d29922;font-size:0.7em;margin-top:4px;}

  /* ─── Buttons ─── */
  .btn{padding:12px 20px;border:none;border-radius:8px;font-size:0.95em;cursor:pointer;margin:4px;transition:all 0.15s ease-in-out;font-weight:500;}
  .btn:active{transform:scale(0.96);filter:brightness(0.9);}
  .btn:focus{outline:2px solid #b84aff;outline-offset:2px;box-shadow:0 0 20px rgba(184,74,255,0.2);}
  .btn-on{background:#238636;color:#fff;}
  .btn-on:hover{background:#2ea043;}
  .btn-off{background:#21262d;border:1px solid #30363d;color:#c9d1d9;}
  .btn-off:hover{background:#30363d;}
  .btn-neutral{background:#30363d;color:#c9d1d9;}
  .btn-neutral:hover{background:#484f58;}
  .btn-random{background:#b84aff;color:#fff;border:none;border-radius:6px;padding:6px 14px;cursor:pointer;font-size:0.85em;font-weight:600;}
  .btn-random:hover{background:#9a3ad9;}
  .btn-reset{background:#30363d;color:#c9d1d9;border:none;border-radius:6px;padding:6px 14px;cursor:pointer;font-size:0.85em;}
  .btn-reset:hover{background:#484f58;}

  /* ─── Config Groups ─── */
  .config-group{margin-bottom:18px;background:#161b22;border:1px solid #2a2a4a;border-radius:12px;padding:16px;box-shadow:0 0 16px rgba(184,74,255,0.06);position:relative;z-index:10;}
  .config-group h3{font-size:0.95em;color:#58a6ff;margin-bottom:12px;border-left:4px solid #58a6ff;padding-left:10px;}
  .config-row{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid #21262d;}
  .config-row:last-child{border-bottom:none;}
  .config-row label{font-size:0.85em;color:#c9d1d9;font-weight:500;}
  .config-row input{width:90px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:8px 12px;font-size:0.95em;text-align:right;font-variant-numeric:tabular-nums;}
  .config-row input:focus{outline:2px solid #58a6ff;outline-offset:2px;}
  .config-row input:invalid{border-color:#da3633;color:#da3633;box-shadow:0 0 8px rgba(218,54,51,0.4);}
  .config-row select{width:160px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:8px 12px;font-size:0.95em;}
  .sticky-save-container{position:sticky;bottom:16px;background:rgba(13,17,23,0.95);padding:12px;border-radius:8px;border:1px solid #2a2a4a;text-align:center;margin-top:16px;box-shadow:0 -4px 12px rgba(0,0,0,0.4),0 0 16px rgba(184,74,255,0.08);z-index:20;}

  /* ─── Logs ─── */
  .log-area{background:#0d1117;border:1px solid #2a2a4a;border-radius:12px;padding:14px;max-height:300px;overflow-y:auto;font-family:monospace;font-size:0.75em;line-height:1.6;box-shadow:0 0 12px rgba(184,74,255,0.06);}
  .log-entry{padding:3px 0;}
  .log-entry.warn{color:#d29922;}
  .log-entry.error{color:#da3633;}

  /* ─── Calibration ─── */
  .calibration-panel{text-align:center;padding:24px;}
  .countdown{font-size:3em;font-weight:bold;color:#58a6ff;}
  .sim-result{background:#161b22;border:1px solid #2a2a4a;border-radius:12px;padding:16px;margin-top:12px;text-align:center;box-shadow:0 0 12px rgba(184,74,255,0.06);}
  .sim-result .time{font-size:1.5em;color:#3fb950;}
  .override-panel{display:none;background:#3a2a1a;color:#d29922;padding:10px;border-radius:8px;margin-bottom:14px;text-align:center;font-weight:bold;border:1px solid #d29922;}

  /* ─── Scrollbar ─── */
  ::-webkit-scrollbar{width:6px;height:6px;}
  ::-webkit-scrollbar-track{background:transparent;}
  ::-webkit-scrollbar-thumb{background:#3d444d;border-radius:10px;}
  ::-webkit-scrollbar-thumb:hover{background:#58a6ff;}

  /* ─── Mascot ─── */
  .mascot-stage{width:64px;height:56px;flex-shrink:0;}
  .mushroom{position:relative;width:64px;height:56px;image-rendering:pixelated;animation:idleBounce 2s ease-in-out infinite;}
  .pixel{position:absolute;width:4px;height:4px;}
  .c1{background:#b02020;}.c2{background:#c83838;}.c3{background:#d85555;}.c4{background:#b84860;}.c5{background:#9a4a70;}.c6{background:#8a3030;}
  .cs{background:#a02020;}.w{background:#ffffff;}.st{background:#f5e6d0;}.sts{background:#e8d5b8;}.bk{background:#1a1a1a;}.bl{background:#f0a0a0;}
  @keyframes idleBounce{0%,15%{transform:translateY(0);}20%{transform:translateY(-2px) scaleY(0.95) scaleX(1.05);}25%{transform:translateY(-4px) scaleY(1.05) scaleX(0.95);}30%{transform:translateY(-2px) scaleY(1.02) scaleX(0.98);}35%{transform:translateY(0) scaleY(0.98) scaleX(1.02);}38%,100%{transform:translateY(0) scaleY(1.0) scaleX(1.0);}}
  @media (prefers-reduced-motion:reduce){.mushroom{animation:none;}}

  /* ─── Warmup ─── */
  #warmupPanel{display:block;background:linear-gradient(180deg,#161b22 0%,#0d1117 100%);border:1px solid #2a2a4a;border-radius:12px;padding:20px;margin:16px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.35),0 0 20px rgba(184,74,255,0.08);position:relative;z-index:10;}
  #warmupProgressBar{width:0%;height:100%;background:linear-gradient(90deg,#58a6ff,#3fb950,#b84aff);transition:width 0.5s linear;}

  /* ─── Mushroom Footer ─── */
  .mushroom-footer{width:100%;max-width:820px;margin:20px auto 0;display:flex;justify-content:center;align-items:flex-end;gap:20px;padding:20px 10px 30px;background:linear-gradient(to top,rgba(0,0,0,0.4) 0%,transparent 100%);flex-wrap:wrap;border-top:1px solid #1a1a2e;position:relative;z-index:20;}
  .mushroom-wrapper{display:flex;flex-direction:column;align-items:center;position:relative;}
  .mushroom-sprite{width:64px;height:64px;position:relative;transform-origin:bottom center;image-rendering:pixelated;}
  .mushroom-sprite .pixel{position:absolute;width:4px;height:4px;shape-rendering:crispEdges;}
  .mushroom-shadow{width:40px;height:10px;background:radial-gradient(ellipse at center,rgba(0,0,0,0.7) 0%,rgba(0,0,0,0) 70%);margin-top:-2px;border-radius:50%;transform-origin:center;}
  @keyframes mushroom-bounce{0%,100%{transform:translateY(0) scaleY(1) scaleX(1);}10%{transform:translateY(2px) scaleY(0.9) scaleX(1.1);}30%{transform:translateY(-12px) scaleY(1.1) scaleX(0.95);}50%{transform:translateY(-16px) scaleY(1.05) scaleX(0.98);}70%{transform:translateY(-8px) scaleY(1.08) scaleX(0.96);}85%{transform:translateY(2px) scaleY(0.95) scaleX(1.05);}95%{transform:translateY(0) scaleY(1) scaleX(1);}}
  @keyframes mushroom-shadow-scale{0%,100%{transform:scale(1);opacity:0.7;}10%{transform:scale(1.15);opacity:0.9;}30%{transform:scale(0.5);opacity:0.25;}50%{transform:scale(0.45);opacity:0.2;}70%{transform:scale(0.55);opacity:0.35;}85%{transform:scale(1.15);opacity:0.9;}95%{transform:scale(1);opacity:0.7;}}
  .mushroom-wrapper:nth-child(1) .mushroom-sprite{animation:mushroom-bounce 2.1s ease-in-out infinite;animation-delay:0s;}
  .mushroom-wrapper:nth-child(1) .mushroom-shadow{animation:mushroom-shadow-scale 2.1s ease-in-out infinite;animation-delay:0s;}
  .mushroom-wrapper:nth-child(2) .mushroom-sprite{animation:mushroom-bounce 2.4s ease-in-out infinite;animation-delay:0.3s;}
  .mushroom-wrapper:nth-child(2) .mushroom-shadow{animation:mushroom-shadow-scale 2.4s ease-in-out infinite;animation-delay:0.3s;}
  .mushroom-wrapper:nth-child(3) .mushroom-sprite{animation:mushroom-bounce 1.9s ease-in-out infinite;animation-delay:0.7s;}
  .mushroom-wrapper:nth-child(3) .mushroom-shadow{animation:mushroom-shadow-scale 1.9s ease-in-out infinite;animation-delay:0.7s;}
  .mushroom-wrapper:nth-child(4) .mushroom-sprite{animation:mushroom-bounce 2.6s ease-in-out infinite;animation-delay:0.1s;}
  .mushroom-wrapper:nth-child(4) .mushroom-shadow{animation:mushroom-shadow-scale 2.6s ease-in-out infinite;animation-delay:0.1s;}
  .mushroom-wrapper:nth-child(5) .mushroom-sprite{animation:mushroom-bounce 2.2s ease-in-out infinite;animation-delay:0.5s;}
  .mushroom-wrapper:nth-child(5) .mushroom-shadow{animation:mushroom-shadow-scale 2.2s ease-in-out infinite;animation-delay:0.5s;}
  @media(max-width:600px){.mushroom-footer{gap:8px;padding:16px 6px 20px;}.mushroom-sprite{transform:scale(0.7);transform-origin:bottom center;}.mushroom-shadow{transform:scale(0.7);}}

  /* ─── Mascots Tab Designer ─── */
  .mascots-tab .designer-grid{display:grid;grid-template-columns:1fr 1fr;gap:20px;}
  @media(max-width:700px){.mascots-tab .designer-grid{grid-template-columns:1fr;}}
  .mascots-tab .designer-preview{background:#161b22;border:1px solid #2a2a4a;border-radius:12px;padding:20px;display:flex;flex-direction:column;align-items:center;}
  .mascots-tab .designer-preview h4{color:#8b949e;font-size:0.8em;text-transform:uppercase;letter-spacing:1px;margin-bottom:12px;}
  .mascots-tab .preview-stage{width:200px;height:200px;display:flex;align-items:flex-end;justify-content:center;background:radial-gradient(ellipse at center bottom,rgba(0,0,0,0.4),transparent);border-radius:16px;}
  .mascots-tab .preview-stage .mushroom-sprite{position:relative;width:60px;height:72px;image-rendering:pixelated;transform-origin:bottom center;}
  .mascots-tab .preview-stage .mushroom-sprite .pixel{position:absolute;border-radius:1px;shape-rendering:crispEdges;}
  @keyframes designerBounce{0%,100%{transform:translateY(0) scaleY(1) scaleX(1);}10%{transform:translateY(2px) scaleY(0.9) scaleX(1.1);}30%{transform:translateY(-12px) scaleY(1.1) scaleX(0.95);}50%{transform:translateY(-16px) scaleY(1.05) scaleX(0.98);}70%{transform:translateY(-8px) scaleY(1.08) scaleX(0.96);}85%{transform:translateY(2px) scaleY(0.95) scaleX(1.05);}95%{transform:translateY(0) scaleY(1) scaleX(1);}}
  @media(prefers-reduced-motion:reduce){.mascots-tab .preview-stage .mushroom-sprite{animation:none !important;}}
  .mascots-tab .preview-name{color:#58a6ff;font-size:1.1em;margin-top:12px;font-weight:bold;}
  .mascots-tab .designer-controls{background:#161b22;border:1px solid #2a2a4a;border-radius:12px;padding:16px;display:flex;flex-direction:column;gap:12px;}
  .mascots-tab .control-group{border-bottom:1px solid #21262d;padding-bottom:12px;}
  .mascots-tab .control-group:last-child{border-bottom:none;padding-bottom:0;}
  .mascots-tab .control-group label{font-size:0.75em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;display:block;margin-bottom:6px;}
  .mascots-tab .btn-group{display:flex;gap:4px;flex-wrap:wrap;}
  .mascots-tab .btn-group button{padding:4px 12px;border:1px solid #30363d;border-radius:6px;background:#0d1117;color:#c9d1d9;cursor:pointer;font-size:0.8em;transition:all 0.2s;}
  .mascots-tab .btn-group button:hover{background:#1c2333;}
  .mascots-tab .btn-group button.active{background:#b84aff;color:#fff;border-color:#b84aff;}
  .mascots-tab .slider-row{display:flex;align-items:center;gap:8px;margin-bottom:4px;}
  .mascots-tab .slider-row span:first-child{font-size:0.8em;color:#8b949e;width:50px;}
  .mascots-tab .slider-row input[type=range]{flex:1;accent-color:#b84aff;background:#21262d;height:4px;border-radius:4px;cursor:pointer;}
  .mascots-tab .slider-row .slider-value{font-size:0.7em;color:#8b949e;width:40px;text-align:right;}
  .mascots-tab .color-row{display:flex;align-items:center;gap:8px;}
  .mascots-tab .color-row>span{font-size:0.8em;color:#8b949e;width:50px;}
  .mascots-tab .color-picker{display:flex;gap:4px;flex-wrap:wrap;align-items:center;}
  .mascots-tab .color-swatch{width:24px;height:24px;border-radius:6px;border:2px solid transparent;cursor:pointer;transition:border 0.2s;position:relative;}
  .mascots-tab .color-swatch:hover{border-color:#58a6ff;}
  .mascots-tab .color-swatch.active{border-color:#b84aff;}
  .mascots-tab .color-swatch.custom{background:transparent !important;border:2px dashed #30363d;display:flex;align-items:center;justify-content:center;}
  .mascots-tab .color-swatch.custom input[type=color]{position:absolute;top:0;left:0;width:100%;height:100%;opacity:0;cursor:pointer;}
  .mascots-tab .color-swatch.custom span{font-size:12px;line-height:1;}
  .mascots-tab .profile-section{border-bottom:1px solid #21262d;padding-bottom:12px;}
  .mascots-tab .profile-row{display:flex;gap:4px;margin-bottom:4px;flex-wrap:wrap;}
  .mascots-tab .profile-row input[type=text]{flex:1;min-width:100px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:4px 8px;font-size:0.85em;}
  .mascots-tab .profile-row select{flex:1;min-width:100px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:4px 8px;font-size:0.85em;}
  .mascots-tab .btn-save-profile{background:#238636;color:#fff;border:none;border-radius:6px;padding:4px 12px;cursor:pointer;font-size:0.8em;}
  .mascots-tab .btn-save-profile:hover{background:#2ea043;}
  .mascots-tab .btn-save-profile:disabled{opacity:0.5;cursor:not-allowed;}
  .mascots-tab .btn-load-profile{background:#58a6ff;color:#fff;border:none;border-radius:6px;padding:4px 12px;cursor:pointer;font-size:0.8em;}
  .mascots-tab .btn-load-profile:hover{background:#79c0ff;}
  .mascots-tab .btn-delete-profile{background:#da3633;color:#fff;border:none;border-radius:6px;padding:4px 12px;cursor:pointer;font-size:0.8em;}
  .mascots-tab .btn-delete-profile:hover{background:#f85149;}
  .mascots-tab .action-row{display:flex;gap:8px;flex-wrap:wrap;padding-top:4px;}
  .mascots-tab .export-area{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:8px;font-family:monospace;font-size:0.65em;max-height:120px;overflow:auto;white-space:pre-wrap;color:#8b949e;display:none;margin-top:6px;}
  .mascots-tab .export-area.show{display:block;}
  .mascots-tab .import-area{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:8px;font-family:monospace;font-size:0.65em;max-height:120px;overflow:auto;color:#8b949e;display:none;margin-top:6px;width:100%;resize:vertical;min-height:60px;}
  .mascots-tab .import-area.show{display:block;}
  .mascots-tab .profile-count{color:#b84aff;font-size:0.85em;}

  /* ─── Background Stars Container ─── */
  #starContainer {
    position: fixed;
    top: 0;
    left: 0;
    width: 100%;
    height: 100vh;
    height: 100dvh;
    z-index: 1;
    pointer-events: none;
    overflow: hidden;
  }

  /* ─── Star Layers ─── */
  .star-drift-wrapper {
    position: fixed;
    pointer-events: none;
    z-index: 1;
    animation: var(--star-drift-name, none) var(--drift-duration, 30s) ease-in-out infinite;
  }

  .star-spin-wrapper {
    width: 100%;
    height: 100%;
    animation: starSpin var(--spin-duration, 60s) linear infinite;
    animation-direction: var(--spin-direction, normal);
  }

  .star-pulse-layer {
    width: 100%;
    height: 100%;
    animation: starPulse var(--pulse-duration, 4s) ease-in-out infinite;
  }

  .star-sprite {
    width: 100%;
    height: 100%;
    position: relative;
    opacity: var(--star-opacity, 0.2);
    image-rendering: pixelated;
  }

  .star-sprite .pixel-grid {
    width: 1px;
    height: 1px;
    position: absolute;
    top: 0;
    left: 0;
    transform-origin: top left;
    border-radius: 0;
  }

  /* ─── Star Keyframes ─── */
  @keyframes starSpin {
    0%   { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }

  @keyframes starPulse {
    0%, 100% { transform: scale(1); }
    50%      { transform: scale(1.15); }
  }

  /* ─── Reduced Motion ─── */
  @media (prefers-reduced-motion: reduce) {
    #starContainer { display: none; }
    .star-drift-wrapper,
    .star-spin-wrapper,
    .star-pulse-layer {
      animation: none !important;
      transform: none !important;
    }
  }

  /* ─── Mobile ─── */
  @media (max-width: 480px) {
    #starContainer { display: none; }
  }
</style>
<script src="/chart-4.4.0.min.js"></script>
</head>
<body>
<div class="header" style="overflow-x:hidden;">
    <div style="display:flex;align-items:center;gap:12px;">
        <div class="mascot-stage" aria-hidden="true"><div class="mushroom" aria-hidden="true">
    <div class="pixel c2" style="top:0;left:24px"></div>
    <div class="pixel c2" style="top:4px;left:16px"></div>
    <div class="pixel c2" style="top:4px;left:20px"></div>
    <div class="pixel c2" style="top:4px;left:24px"></div>
    <div class="pixel c3" style="top:4px;left:28px"></div>
    <div class="pixel c3" style="top:4px;left:32px"></div>
    <div class="pixel c2" style="top:4px;left:36px"></div>
    <div class="pixel c2" style="top:4px;left:40px"></div>
    <div class="pixel c2" style="top:4px;left:44px"></div>
    <div class="pixel c6" style="top:8px;left:12px"></div>
    <div class="pixel c2" style="top:8px;left:16px"></div>
    <div class="pixel w" style="top:8px;left:20px"></div>
    <div class="pixel c3" style="top:8px;left:24px"></div>
    <div class="pixel c3" style="top:8px;left:28px"></div>
    <div class="pixel c2" style="top:8px;left:32px"></div>
    <div class="pixel w" style="top:8px;left:36px"></div>
    <div class="pixel c3" style="top:8px;left:40px"></div>
    <div class="pixel c2" style="top:8px;left:44px"></div>
    <div class="pixel c6" style="top:8px;left:48px"></div>
    <div class="pixel c4" style="top:12px;left:8px"></div>
    <div class="pixel c6" style="top:12px;left:12px"></div>
    <div class="pixel w" style="top:12px;left:16px"></div>
    <div class="pixel c3" style="top:12px;left:20px"></div>
    <div class="pixel c6" style="top:12px;left:24px"></div>
    <div class="pixel w" style="top:12px;left:28px"></div>
    <div class="pixel c3" style="top:12px;left:32px"></div>
    <div class="pixel c6" style="top:12px;left:36px"></div>
    <div class="pixel w" style="top:12px;left:40px"></div>
    <div class="pixel c3" style="top:12px;left:44px"></div>
    <div class="pixel c4" style="top:12px;left:48px"></div>
    <div class="pixel c4" style="top:12px;left:52px"></div>
    <div class="pixel c6" style="top:16px;left:8px"></div>
    <div class="pixel c3" style="top:16px;left:12px"></div>
    <div class="pixel w" style="top:16px;left:16px"></div>
    <div class="pixel c6" style="top:16px;left:20px"></div>
    <div class="pixel c3" style="top:16px;left:24px"></div>
    <div class="pixel w" style="top:16px;left:28px"></div>
    <div class="pixel c6" style="top:16px;left:32px"></div>
    <div class="pixel c3" style="top:16px;left:36px"></div>
    <div class="pixel w" style="top:16px;left:40px"></div>
    <div class="pixel c6" style="top:16px;left:44px"></div>
    <div class="pixel c3" style="top:16px;left:48px"></div>
    <div class="pixel c4" style="top:16px;left:52px"></div>
    <div class="pixel c6" style="top:20px;left:4px"></div>
    <div class="pixel c6" style="top:20px;left:8px"></div>
    <div class="pixel c6" style="top:20px;left:12px"></div>
    <div class="pixel c3" style="top:20px;left:16px"></div>
    <div class="pixel c6" style="top:20px;left:20px"></div>
    <div class="pixel c3" style="top:20px;left:24px"></div>
    <div class="pixel c6" style="top:20px;left:28px"></div>
    <div class="pixel c3" style="top:20px;left:32px"></div>
    <div class="pixel c6" style="top:20px;left:36px"></div>
    <div class="pixel c3" style="top:20px;left:40px"></div>
    <div class="pixel c6" style="top:20px;left:44px"></div>
    <div class="pixel c6" style="top:20px;left:48px"></div>
    <div class="pixel c6" style="top:20px;left:52px"></div>
    <div class="pixel c6" style="top:20px;left:56px"></div>
    <div class="pixel c6" style="top:24px;left:4px"></div>
    <div class="pixel c6" style="top:24px;left:8px"></div>
    <div class="pixel c6" style="top:24px;left:12px"></div>
    <div class="pixel c6" style="top:24px;left:16px"></div>
    <div class="pixel c6" style="top:24px;left:20px"></div>
    <div class="pixel c6" style="top:24px;left:24px"></div>
    <div class="pixel c6" style="top:24px;left:28px"></div>
    <div class="pixel c6" style="top:24px;left:32px"></div>
    <div class="pixel c6" style="top:24px;left:36px"></div>
    <div class="pixel c6" style="top:24px;left:40px"></div>
    <div class="pixel c6" style="top:24px;left:44px"></div>
    <div class="pixel c6" style="top:24px;left:48px"></div>
    <div class="pixel c6" style="top:24px;left:52px"></div>
    <div class="pixel c6" style="top:24px;left:56px"></div>
    <div class="pixel st" style="top:28px;left:24px"></div>
    <div class="pixel st" style="top:28px;left:28px"></div>
    <div class="pixel st" style="top:28px;left:32px"></div>
    <div class="pixel st" style="top:28px;left:36px"></div>
    <div class="pixel sts" style="top:32px;left:24px"></div>
    <div class="pixel bk" style="top:32px;left:28px"></div>
    <div class="pixel st" style="top:32px;left:32px"></div>
    <div class="pixel bk" style="top:32px;left:36px"></div>
    <div class="pixel sts" style="top:32px;left:40px"></div>
    <div class="pixel sts" style="top:36px;left:20px"></div>
    <div class="pixel st" style="top:36px;left:24px"></div>
    <div class="pixel bl" style="top:36px;left:28px"></div>
    <div class="pixel st" style="top:36px;left:32px"></div>
    <div class="pixel bl" style="top:36px;left:36px"></div>
    <div class="pixel st" style="top:36px;left:40px"></div>
    <div class="pixel sts" style="top:36px;left:44px"></div>
    <div class="pixel sts" style="top:40px;left:22px"></div>
    <div class="pixel sts" style="top:40px;left:26px"></div>
    <div class="pixel st" style="top:40px;left:30px"></div>
    <div class="pixel st" style="top:40px;left:34px"></div>
    <div class="pixel sts" style="top:40px;left:38px"></div>
    <div class="pixel sts" style="top:44px;left:24px"></div>
    <div class="pixel sts" style="top:44px;left:28px"></div>
    <div class="pixel sts" style="top:44px;left:32px"></div>
    <div class="pixel sts" style="top:44px;left:36px"></div>
</div></div>
        <div style="flex:1;">
            <div style="display:flex;align-items:center;justify-content:space-between;">
                <h1 style="margin:0;line-height:1.2;">GrowHub32</h1>
                <div id="warmupBadge" style="display:none;background:#58a6ff;color:#0d1117;padding:4px 12px;border-radius:20px;font-size:0.7em;font-weight:bold;animation:pulse-blue 1.5s infinite;">⚙️ Warming Up</div>
            </div>
            <div class="status" id="connectionStatus" style="margin-top:2px;">Connecting...</div>
            <div id="warmupProgressContainer" style="display:none;margin-top:4px;width:100%;height:4px;background:#21262d;border-radius:4px;overflow:hidden;">
                <div id="warmupProgressBar" style="width:0%;height:100%;background:linear-gradient(90deg,#58a6ff,#3fb950,#b84aff);transition:width 0.5s linear;"></div>
            </div>
        </div>
    </div>
</div>

<div class="warning-banner" id="warningBanner">Warning: Sensor Fault - System Running Last Known Values</div>
<div id="toastPanel" style="display:none;position:fixed;bottom:80px;left:50%;transform:translateX(-50%);background:#30363d;color:#e6edf3;padding:10px 24px;border-radius:20px;font-size:0.9em;font-weight:500;z-index:200;box-shadow:0 4px 12px rgba(0,0,0,0.5);pointer-events:none;"><span id="toastMsg"></span></div>

<div class="tabs">
  <button class="tab active" onclick="switchTab(this, 'dashboard')">Dashboard</button>
  <button class="tab" onclick="switchTab(this, 'controls')">Controls</button>
  <button class="tab" onclick="switchTab(this, 'config')">Config</button>
  <button class="tab" onclick="switchTab(this, 'calibration')">Calibrate</button>
  <button class="tab" onclick="switchTab(this, 'simulation')">Simulate</button>
  <button class="tab" onclick="switchTab(this, 'graphs')">Graphs</button>
  <button class="tab" onclick="switchTab(this, 'logs')">Logs</button>
  <button class="tab" onclick="switchTab(this, 'mascots')">🎨 Mascots</button>
</div>

<!-- Warmup panel -->
<div id="warmupPanel" style="display:block;background:linear-gradient(180deg,#161b22 0%,#0d1117 100%);border:1px solid #2a2a4a;border-radius:12px;padding:20px;margin:16px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.35),0 0 20px rgba(184,74,255,0.08);">
    <h3 style="color:#58a6ff;margin-bottom:8px;">🔄 Compressor Warmup</h3>
    <p style="font-size:0.85em;color:#8b949e;margin-bottom:12px;">The air tank needs time to fill before Air Assist can work. How long should the compressor warm up?</p>
    <div style="display:flex;gap:8px;justify-content:center;flex-wrap:wrap;">
        <button class="btn btn-neutral" onclick="startWarmup(0)">Skip</button>
        <button class="btn btn-neutral" onclick="startWarmup(30)">30 sec</button>
        <button class="btn btn-neutral" onclick="startWarmup(60)">1 min</button>
        <button class="btn btn-neutral" onclick="startWarmup(120)">2 min</button>
        <button class="btn btn-neutral" onclick="startWarmup(300)">5 min</button>
    </div>
</div>

<!-- Dashboard -->
<div id="dashboard" class="tab-content active">
  <div class="sensor-grid">
    <div class="sensor-card">
      <div class="label"><span class="status-dot ok" id="tempDot"></span>Temperature</div>
      <div class="value" id="tempValue">--</div>
      <div class="unit">C</div>
    </div>
    <div class="sensor-card">
      <div class="label"><span class="status-dot ok" id="humDot"></span>Humidity</div>
      <div class="value" id="humValue">--</div>
      <div class="unit">% RH</div>
    </div>
    <div class="sensor-card">
      <div class="label"><span class="status-dot ok" id="co2Dot"></span>CO2</div>
      <div class="value" id="co2Value">--</div>
      <div class="unit">ppm</div>
    </div>
    <div class="sensor-card">
      <div class="label">Fridge</div>
      <div class="value" id="fridgeValue">--</div>
      <div class="unit">C / <span id="fridgeHumValue">--</span>%</div>
    </div>
    <div class="sensor-card">
      <div class="label">Fridge Door</div>
      <div class="value" id="fridgeDoorValue">--</div>
      <div class="unit"></div>
    </div>
  </div>
   <div class="config-group" id="weatherPanel" style="display:none;">
    <h3>🌤️ Outdoor Weather</h3>
    <div style="display:flex;align-items:center;gap:12px;flex-wrap:wrap;">
      <span id="weatherIcon" style="font-size:2.5em;" aria-label="Current weather condition"></span>
      <div>
        <div id="weatherTemp" style="font-size:1.4em;font-weight:bold;color:#ffffff;">--</div>
        <div id="weatherHum" style="font-size:0.85em;color:#8b949e;">--</div>
        <div id="weatherWind" style="font-size:0.85em;color:#8b949e;">--</div>
      </div>
    </div>
    <div id="weatherStale" style="display:none;font-size:0.75em;color:#d29922;margin-top:6px;">⚠️ Data may be stale</div>
  </div>
  <div class="relay-grid">
       <div class="relay-card" data-relay-index="0" data-relay-state="false"><div class="name">Humidifier</div><div class="state off" id="hohState">OFF</div></div>
    <div class="relay-card" data-relay-index="1" data-relay-state="false"><div class="name">Air Assist</div><div class="state off" id="assistState">OFF</div></div>
    <div class="relay-card" data-relay-index="2" data-relay-state="false"><div class="name">Exhaust Fan</div><div class="state off" id="fanState">OFF</div></div>
    <div class="relay-card" data-relay-index="3" data-relay-state="false"><div class="name">Compressor</div><div class="state off" id="compState">OFF</div><div class="locked" id="compLock"></div></div>
    </div>
  <div class="config-group" id="alertsPanel" style="display:none;">
    <h3>⚠️ Alerts</h3>
    <div id="alertsList"></div>
  </div>
  <div class="config-group">
    <h3>System Status</h3>
    <div class="config-row"><label>Night Mode</label><span id="nightModeStatus">--</span></div>
    <div class="config-row"><label>WiFi</label><span id="wifiStatus">--</span></div>
    <div class="config-row"><label>RTC Time</label><span id="rtcTime">--</span></div>
    <div class="config-row"><label>Fridge Node</label><span id="fridgeStatus">--</span></div>
    <div class="config-row"><label>Control Mode</label><span id="controlMode">--</span></div>
  </div>
</div>

<!-- Controls -->
<div id="controls" class="tab-content">
  <h3>Manual Relay Override</h3>
  <p style="font-size:0.75em;color:#8b949e;">Calibration mode must be OFF to use manual controls. Manual commands pause automation. Safety interlocks remain active.</p>
  <div class="override-panel" id="overridePanel">
    Automation PAUSED - <span id="overrideTime">0:00</span> remaining
    <br><button class="btn btn-off" style="margin-top:6px;" onclick="resumeAutomation()">Resume Automation Now</button>
  </div>
  <div class="config-group">
    <div class="config-row"><label>Humidifier</label><div><button class="btn btn-on" onclick="relayCmd(0,1)">ON</button><button class="btn btn-off" onclick="relayCmd(0,0)">OFF</button></div></div>
    <div class="config-row"><label>Air Assist</label><div><button class="btn btn-on" onclick="relayCmd(1,1)">ON</button><button class="btn btn-off" onclick="relayCmd(1,0)">OFF</button></div></div>
    <div class="config-row"><label>Exhaust Fan</label><div><button class="btn btn-on" onclick="relayCmd(2,1)">ON</button><button class="btn btn-off" onclick="relayCmd(2,0)">OFF</button></div></div>
    <div class="config-row"><label>Compressor</label><div><button class="btn btn-on" onclick="relayCmd(3,1)">ON</button><button class="btn btn-off" onclick="relayCmd(3,0)">OFF</button></div></div>
  </div>
</div>

<!-- Config -->
<div id="config" class="tab-content">
  <div class="config-group">
    <h3>Humidity Thresholds</h3>
    <div class="config-row"><label>HOH Floor (%)</label><input type="number" id="humHoHFloor" value="80" step="1"></div>
    <div class="config-row"><label>Assist Floor (%)</label><input type="number" id="humAssistFloor" value="70" step="1"></div>
    <div class="config-row"><label>Exhaust ON (%)</label><input type="number" id="humExhaustOn" value="92" step="1"></div>
    <div class="config-row"><label>Ceiling (%)</label><input type="number" id="humCeiling" value="88" step="1"></div>
    <div class="config-row"><label>Assist ON (sec)</label><input type="number" id="assistOn" value="3" step="1"></div>
    <div class="config-row"><label>Assist OFF (sec)</label><input type="number" id="assistOff" value="10" step="1"></div>
  </div>
  <div class="config-group">
    <h3>CO2 Thresholds</h3>
    <div class="config-row"><label>High Limit (ppm)</label><input type="number" id="co2High" value="800" step="10"></div>
    <div class="config-row"><label>Low Target (ppm)</label><input type="number" id="co2Low" value="600" step="10"></div>
    <div class="config-row"><label>Emergency (ppm)</label><input type="number" id="co2Emergency" value="1200" step="10"></div>
  </div>
  <div class="config-group">
    <h3>Real-Time Clock</h3>
    <div class="config-row"><label>Set Date/Time</label><input type="datetime-local" id="rtcDateTime"></div>
    <button class="btn btn-on" onclick="setRTCTime()">Set RTC Time</button>
  </div>
  <div class="config-group">
    <h3>Adaptive Learning</h3>
    <div class="config-row"><label>EMA Weight (0.10-0.50)</label><input type="number" id="emaWeight" value="0.30" step="0.05" min="0.10" max="0.50"></div>
  </div>
   <div class="config-group">
    <h3>Manual Override Timeout</h3>
    <div class="config-row"><label>Timeout (minutes)</label><input type="number" id="overrideTimeout" value="10" min="1" max="1440" step="1"></div>
    <div style="display:flex;gap:4px;margin-top:8px;flex-wrap:wrap;">
      <button class="btn btn-neutral override-btn" onclick="setOverrideTime(10,this)">10 min</button>
      <button class="btn btn-neutral override-btn" onclick="setOverrideTime(30,this)">30 min</button>
      <button class="btn btn-neutral override-btn" onclick="setOverrideTime(60,this)">1 hr</button>
      <button class="btn btn-neutral override-btn" onclick="setOverrideTime(360,this)">6 hr</button>
      <button class="btn btn-neutral override-btn" onclick="setOverrideTime(1440,this)">24 hr</button>
    </div>
  </div>
  <div class="config-group">
    <h3>Weather Location</h3>
    <div class="config-row"><label>Latitude</label><input type="number" id="weatherLat" value="43.68" step="0.01" min="-90" max="90"></div>
    <div class="config-row"><label>Longitude</label><input type="number" id="weatherLon" value="-79.77" step="0.01" min="-180" max="180"></div>
  </div>
  <div class="config-group">
    <h3>Relay Mapping</h3>
    <p style="font-size:0.7em;color:#8b949e;margin-bottom:8px;">Assign functions to physical relay positions 1-4. Use the ID button next to each position to identify which relay is which. Each function must be assigned exactly once.</p>
    <div class="config-row"><label>Position 1</label><div style="display:flex;align-items:center;gap:8px;"><button class="btn btn-neutral" onclick="identifyRelay(0)" style="padding:4px 10px;font-size:0.75em;">ID</button><select id="funcPos1"><option value="0" selected>HOH Humidifier</option><option value="1">Air Assist</option><option value="2">Exhaust Fan</option><option value="3">Compressor</option></select></div></div>
    <div class="config-row"><label>Position 2</label><div style="display:flex;align-items:center;gap:8px;"><button class="btn btn-neutral" onclick="identifyRelay(1)" style="padding:4px 10px;font-size:0.75em;">ID</button><select id="funcPos2"><option value="0">HOH Humidifier</option><option value="1" selected>Air Assist</option><option value="2">Exhaust Fan</option><option value="3">Compressor</option></select></div></div>
    <div class="config-row"><label>Position 3</label><div style="display:flex;align-items:center;gap:8px;"><button class="btn btn-neutral" onclick="identifyRelay(2)" style="padding:4px 10px;font-size:0.75em;">ID</button><select id="funcPos3"><option value="0">HOH Humidifier</option><option value="1">Air Assist</option><option value="2" selected>Exhaust Fan</option><option value="3">Compressor</option></select></div></div>
    <div class="config-row"><label>Position 4</label><div style="display:flex;align-items:center;gap:8px;"><button class="btn btn-neutral" onclick="identifyRelay(3)" style="padding:4px 10px;font-size:0.75em;">ID</button><select id="funcPos4"><option value="0">HOH Humidifier</option><option value="1">Air Assist</option><option value="2">Exhaust Fan</option><option value="3" selected>Compressor</option></select></div></div>
    <button class="btn btn-on" onclick="saveRelayMapping()">Apply Relay Mapping</button>
  </div>
  <div class="sticky-save-container">
    <button class="btn btn-on" onclick="saveThresholds()">Save All Thresholds</button>
  </div>
</div>

<!-- Calibration -->
<div id="calibration" class="tab-content">
  <div class="calibration-panel">
    <h3>Calibration Status</h3>
    <div class="countdown" id="calibCountdown">--</div>
    <p id="calibStatus" style="color:#8b949e;">Not active</p>
    <button class="btn btn-on" id="calibStartBtn" onclick="startCalibration()">Start 20-Minute Calibration</button>
    <button class="btn btn-off" id="calibCancelBtn" onclick="cancelCalibration()" style="display:none;">Cancel Calibration</button>
  </div>
</div>

<!-- Simulation -->
<div id="simulation" class="tab-content">
  <h3>Recovery Simulation</h3>
  <div class="config-group">
    <div class="config-row"><label>Current RH (%)</label><span id="simCurrentRH">--</span></div>
    <div class="config-row"><label>Target RH (%)</label><input type="number" id="simTargetRH" value="88" step="1"></div>
    <div class="config-row"><label>Active Band</label><span id="simBand">--</span></div>
    <div class="config-row"><label>Confidence</label><span id="simConfidence">--</span></div>
  </div>
  <button class="btn btn-neutral" onclick="runSimulation()">Run Simulation</button>
  <div class="sim-result">
    <div>Projected Recovery Time:</div>
    <div class="time" id="simResult">--</div>
  </div>
</div>

<!-- Graphs -->
<div id="graphs" class="tab-content">
  <div style="display:flex;gap:6px;margin-bottom:12px;flex-wrap:wrap;">
    <button class="btn btn-neutral graph-tab active" onclick="switchGraphTab(this,0)">Temp</button>
    <button class="btn btn-neutral graph-tab" onclick="switchGraphTab(this,1)">Humidity</button>
    <button class="btn btn-neutral graph-tab" onclick="switchGraphTab(this,2)">CO2</button>
    <button class="btn btn-neutral graph-tab" onclick="switchGraphTab(this,3)">Fridge</button>
    <span style="flex:1;"></span>
    <button class="btn btn-neutral graph-range-btn" onclick="setGraphRange(3600,this)">1h</button>
    <button class="btn btn-neutral graph-range-btn" onclick="setGraphRange(21600,this)">6h</button>
    <button class="btn btn-neutral graph-range-btn active" onclick="setGraphRange(86400,this)">24h</button>
  </div>
  <div style="position:relative;width:100%;height:350px;">
    <canvas id="graphCanvas"></canvas>
  </div>
</div>

<!-- Logs -->
<div id="logs" class="tab-content">
  <h3>System Log</h3>
  <div class="log-area" id="logArea">
    <div class="log-entry">Waiting for data...</div>
  </div>
</div>

<!-- ─── MASCOTS TAB ─── -->
<div id="mascots" class="tab-content mascots-tab">
  <h2 style="color:#58a6ff;margin-bottom:16px;">🍄 Mushroom Designer</h2>
  <p style="color:#8b949e;font-size:0.85em;margin-bottom:16px;">
    Design your own custom mushroom mascots. Adjust the sliders, pick colours, and save your favourites.
    <span class="profile-count" id="profileCount">(0/8 profiles used)</span>
  </p>

  <div class="designer-grid">
    <!-- Preview -->
    <div class="designer-preview">
      <h4>📺 Live Preview</h4>
      <div class="preview-stage">
        <div class="mushroom-sprite" id="designerSprite"></div>
      </div>
      <div class="preview-name" id="designerName">Amanita</div>
    </div>

    <!-- Controls -->
    <div class="designer-controls">
      <!-- Template -->
      <div class="control-group">
        <label>📋 Template</label>
        <div class="btn-group" id="templateButtons">
          <button class="active" data-template="amanita">Amanita</button>
          <button data-template="chanterelle">Chanterelle</button>
          <button data-template="shiitake">Shiitake</button>
          <button data-template="magic">Magic</button>
          <button data-template="morel">Morel</button>
        </div>
      </div>

      <!-- Cap -->
      <div class="control-group">
        <label>🔴 Cap</label>
        <div class="slider-row">
          <span>Width</span>
          <input type="range" id="capWidth" min="60" max="120" value="100">
          <span class="slider-value" id="capWidthVal">100%</span>
        </div>
        <div class="slider-row">
          <span>Height</span>
          <input type="range" id="capHeight" min="60" max="120" value="100">
          <span class="slider-value" id="capHeightVal">100%</span>
        </div>
        <div class="color-row">
          <span>Color</span>
          <div class="color-picker" id="capColorPicker">
            <div class="color-swatch active" style="background:#e63946;" data-color="#e63946"></div>
            <div class="color-swatch" style="background:#f4a261;" data-color="#f4a261"></div>
            <div class="color-swatch" style="background:#795548;" data-color="#795548"></div>
            <div class="color-swatch" style="background:#7b2cbf;" data-color="#7b2cbf"></div>
            <div class="color-swatch" style="background:#40916c;" data-color="#40916c"></div>
            <div class="color-swatch" style="background:#ff6b6b;" data-color="#ff6b6b"></div>
            <div class="color-swatch" style="background:#ffd93d;" data-color="#ffd93d"></div>
            <div class="color-swatch" style="background:#6bcbff;" data-color="#6bcbff"></div>
            <div class="color-swatch" style="background:#a66cff;" data-color="#a66cff"></div>
            <div class="color-swatch custom">
              <input type="color" id="customCapColor" value="#e63946">
              <span>🎨</span>
            </div>
          </div>
        </div>
        <div class="btn-group" id="spotButtons">
          <button class="active" data-spots="2">2 Spots</button>
          <button data-spots="4">4 Spots</button>
          <button data-spots="6">6 Spots</button>
          <button data-spots="0">None</button>
        </div>
      </div>

      <!-- Stem -->
      <div class="control-group">
        <label>🌿 Stem</label>
        <div class="slider-row">
          <span>Height</span>
          <input type="range" id="stemHeight" min="60" max="140" value="100">
          <span class="slider-value" id="stemHeightVal">100%</span>
        </div>
        <div class="slider-row">
          <span>Width</span>
          <input type="range" id="stemWidth" min="50" max="120" value="100">
          <span class="slider-value" id="stemWidthVal">100%</span>
        </div>
        <div class="color-row">
          <span>Color</span>
          <div class="color-picker" id="stemColorPicker">
            <div class="color-swatch active" style="background:#fefae0;" data-color="#fefae0"></div>
            <div class="color-swatch" style="background:#f5e6d0;" data-color="#f5e6d0"></div>
            <div class="color-swatch" style="background:#efebe9;" data-color="#efebe9"></div>
            <div class="color-swatch" style="background:#e8d5b8;" data-color="#e8d5b8"></div>
            <div class="color-swatch" style="background:#f0e6d3;" data-color="#f0e6d3"></div>
            <div class="color-swatch custom">
              <input type="color" id="customStemColor" value="#fefae0">
              <span>🎨</span>
            </div>
          </div>
        </div>
      </div>

      <!-- Animation -->
         <!-- Animation -->
      <div class="control-group">
        <label>🌀 Animation</label>
        <div class="slider-row">
          <span>Bounce</span>
          <input type="range" id="bounceHeight" min="5" max="35" value="20">
          <span class="slider-value" id="bounceHeightVal">20px</span>
        </div>
        <div class="slider-row">
          <span>Speed</span>
          <input type="range" id="animSpeed" min="12" max="35" value="22" step="1">
          <span class="slider-value" id="animSpeedVal">2.2s</span>
        </div>
      </div>

      <!-- ─── Face Customization ─── -->
      <div class="control-group">
        <label>😊 Face</label>

        <div style="margin-bottom:4px;">
          <span style="font-size:0.7em;color:#8b949e;">Eyes</span>
          <div class="btn-group">
            <button data-eye="normal" class="active">Normal</button>
            <button data-eye="happy">😄</button>
            <button data-eye="sleepy">😴</button>
            <button data-eye="closed">😌</button>
            <button data-eye="big">👀</button>
            <button data-eye="none">✖</button>
          </div>
        </div>

        <div style="margin-bottom:4px;">
          <span style="font-size:0.7em;color:#8b949e;">Mouth</span>
          <div class="btn-group">
            <button data-mouth="smile" class="active">Smile</button>
            <button data-mouth="happy">😁</button>
            <button data-mouth="neutral">😐</button>
            <button data-mouth="surprised">😮</button>
            <button data-mouth="none">✖</button>
          </div>
        </div>

        <div class="slider-row">
          <span style="font-size:0.7em;color:#8b949e;width:50px;">Blush</span>
          <input type="checkbox" id="blushEnable" checked>
          <label style="font-size:0.7em;color:#8b949e;margin-left:4px;">On</label>
          <input type="color" id="blushColor" value="#ff99c8" style="width:30px;height:24px;border:none;background:transparent;cursor:pointer;margin-left:8px;">
          <span style="font-size:0.7em;color:#8b949e;margin-left:8px;">Size</span>
          <input type="range" id="blushSize" min="5" max="20" value="10" step="1" style="flex:0.5;">
          <span class="slider-value" id="blushSizeVal">1.0</span>
        </div>

        <div style="margin-top:4px;">
          <span style="font-size:0.7em;color:#8b949e;">Cap Texture</span>
          <div class="btn-group">
            <button data-texture="smooth" class="active">Smooth</button>
            <button data-texture="textured">Textured</button>
            <button data-texture="striped">Striped</button>
            <button data-texture="gradient">Gradient</button>
          </div>
        </div>
      </div>

      <!-- Profile -->
      <div class="control-group profile-section">
        <label>💾 Profile</label>
        <div class="profile-row">
          <input type="text" id="profileNameInput" placeholder="Profile name..." maxlength="32">
          <button class="btn-save-profile" id="saveProfileBtn">💾 Save</button>
        </div>
        <div class="profile-row">
          <select id="profileLoadSelect">
            <option value="">-- No saved profiles --</option>
          </select>
          <button class="btn-load-profile" id="loadProfileBtn">📂 Load</button>
          <button class="btn-delete-profile" id="deleteProfileBtn">🗑 Delete</button>
        </div>
        <div style="font-size:0.7em;color:#8b949e;margin-top:4px;">
          Max 8 profiles. Saved designs persist on the device.
        </div>
      </div>

      <!-- Background Stars -->
      <div class="control-group">
        <label>🌌 Background Mushrooms</label>
        <div class="profile-row">
          <label style="font-size:0.8em;color:#8b949e;width:60px;">Enable</label>
          <input type="checkbox" id="starEnable" checked>
        </div>
        <div class="slider-row">
          <span>Size</span>
          <input type="range" id="starSize" min="32" max="150" value="56">
          <span class="slider-value" id="starSizeVal">56px</span>
        </div>
        <div style="display:flex;gap:4px;margin-left:58px;flex-wrap:wrap;">
          <button class="btn-star-size" data-size="32" style="font-size:0.65em;padding:2px 8px;background:#30363d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;cursor:pointer;">Small</button>
          <button class="btn-star-size" data-size="56" style="font-size:0.65em;padding:2px 8px;background:#30363d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;cursor:pointer;">Medium</button>
          <button class="btn-star-size" data-size="80" style="font-size:0.65em;padding:2px 8px;background:#30363d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;cursor:pointer;">Large</button>
          <button class="btn-star-size" data-size="120" style="font-size:0.65em;padding:2px 8px;background:#30363d;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;cursor:pointer;">XL</button>
        </div>
        <div class="slider-row">
          <span>Opacity</span>
          <input type="range" id="starOpacity" min="8" max="45" value="22">
          <span class="slider-value" id="starOpacityVal">22%</span>
        </div>
        <div class="slider-row">
          <span>Drift Speed</span>
          <input type="range" id="starDriftSpeed" min="10" max="45" value="22">
          <span class="slider-value" id="starDriftSpeedVal">22s</span>
        </div>
        <div class="slider-row">
          <span>Spin Speed</span>
          <input type="range" id="starSpinRPM" min="1" max="20" value="6" step="1">
          <span class="slider-value" id="starSpinRPMVal">0.6 RPM</span>
        </div>
        <div class="slider-row">
          <span>Pulse Speed</span>
          <input type="range" id="starPulseSpeed" min="20" max="80" value="40" step="1">
          <span class="slider-value" id="starPulseSpeedVal">4.0s</span>
        </div>
        <div class="slider-row">
          <span>Drift Range</span>
          <input type="range" id="starDriftRange" min="50" max="400" value="200" step="10">
          <span class="slider-value" id="starDriftRangeVal">200px</span>
        </div>
        <div style="display:flex;gap:8px;margin-top:4px;flex-wrap:wrap;">
          <button class="btn-random" id="randomiseStarsBtn">🔄 Randomise Positions</button>
          <button class="btn-reset" id="resetStarsBtn">↺ Recenter Stars</button>
        </div>
      </div>

      <!-- Actions -->
      <div class="action-row">
        <button class="btn-random" id="randomizeBtn">🎲 Randomise</button>
        <button class="btn-reset" id="resetBtn">↺ Reset</button>
        <button class="btn-export" id="exportBtn">📋 Export JSON</button>
        <button class="btn-import" id="importToggleBtn">📥 Import JSON</button>
      </div>
      <textarea class="import-area" id="importArea" placeholder="Paste JSON here to import..."></textarea>
      <div class="export-area" id="exportArea"></div>
    </div>
  </div>
</div>
<!-- ─── END MASCOTS TAB ─── -->

<!-- ─── MUSHROOM FAMILY FOOTER ─── -->
<div class="mushroom-footer" id="mushroomFooter"></div>
<!-- ─── END MUSHROOM FOOTER ─── -->

<script>
// ============================================================
// MAIN DASHBOARD LOGIC
// ============================================================

var ws;
var reconnectDelay = 3000;

// Warmup state
var warmupRemaining = 0;
var warmupDuration = 0;
var warmupInterval = null;
var isWarmupComplete = false;

// Graph globals
var lastRequestedSensor = 0;

function sendWS(data){
  if(ws && ws.readyState === WebSocket.OPEN){
    ws.send(JSON.stringify(data));
    return true;
  }
  return false;
}

function connectWS(){
  if (ws && ws.readyState <= 1) return;
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = function(){
    document.getElementById('connectionStatus').textContent = 'Connected | ' + location.hostname;
    reconnectDelay = 3000;
    initGraph();
    requestHistorical();
    setTimeout(function() { initDesigner(); }, 200);
  };
  ws.onmessage = function(e){
    try{
      var msg = JSON.parse(e.data);
      handleMessage(msg);
    }catch(err){
      console.log('WS parse error:', err);
    }
  };
  ws.onclose = function(){
    document.getElementById('connectionStatus').textContent = 'Disconnected - retrying...';
    setTimeout(connectWS, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 2, 30000);
  };
  ws.onerror = function(){
    if (ws && ws.readyState !== WebSocket.CLOSED) ws.close();
  };
}

function handleMessage(msg){
  switch(msg.type){
    case 0: updateSensors(msg); break;
    case 1: updateRelays(msg); updateOverrideStatus(msg); updateAlerts(msg); break;
    case 2:
      if(msg.message === "CONFIRM_LOUD_NIGHT"){
        var relayNames = ["Humidifier","Air Assist","Exhaust Fan","Compressor"];
        var relayName = relayNames[msg.relay] || "This device";
        if(confirm(relayName + " is loud. Are you sure you want to turn it on during night mode?\n\nIt will run for 10 minutes before night mode lockout resumes.")){
          sendWS({type: 6, cmd: 'relay', index: msg.relay, state: 1, force: true, confirmed: true});
        }
      } else {
        addLog(msg.message, msg.level || 'warn');
      }
      break;
    case 3: updateConfig(msg); break;
    case 4: updateCalibration(msg); break;
    case 5: addLog(msg.message, msg.level || 'info'); break;
    case 7: handleProfileResponse(msg); break;
    case 99:
      if(msg.simResult){
        document.getElementById('simResult').textContent = msg.simResult;
      }
      break;
    case 100: handleGraphResponse(msg); break;
  }
}

function updateSensors(msg){
  document.getElementById('tempValue').textContent = (typeof msg.temp === 'number') ? msg.temp.toFixed(1) : '--';
  document.getElementById('humValue').textContent = (typeof msg.hum === 'number') ? msg.hum.toFixed(1) : '--';
  document.getElementById('co2Value').textContent = (typeof msg.co2 === 'number') ? msg.co2 : '--';
  document.getElementById('fridgeValue').textContent = (typeof msg.fridge === 'number') ? msg.fridge.toFixed(1) : '--';
  document.getElementById('fridgeHumValue').textContent = (typeof msg.fridgeHum === 'number') ? msg.fridgeHum.toFixed(1) : '--';
  var doorEl = document.getElementById('fridgeDoorValue');
  if (msg.fridgeLost) {
    doorEl.textContent = 'OFFLINE';
    doorEl.style.color = '#d29922';
  } else if (msg.fridgeDoor === true) {
    doorEl.textContent = 'OPEN';
    doorEl.style.color = '#da3633';
  } else if (msg.fridgeDoor === false) {
    doorEl.textContent = 'CLOSED';
    doorEl.style.color = '#3fb950';
  } else {
    doorEl.textContent = '--';
    doorEl.style.color = '#8b949e';
  }

  document.getElementById('tempDot').className = 'status-dot ' + (msg.tempFault ? 'fault' : 'ok');
  document.getElementById('humDot').className = 'status-dot ' + (msg.humFault ? 'fault' : 'ok');
  document.getElementById('co2Dot').className = 'status-dot ' + (msg.co2Fault ? 'fault' : 'ok');

  var banner = document.getElementById('warningBanner');
  if(msg.tempFault || msg.humFault || msg.co2Fault){
    banner.className = 'warning-banner active';
    banner.textContent = 'Warning: Sensor Fault - System Running Last Known Values';
  } else {
    banner.className = 'warning-banner';
  }

  document.getElementById('nightModeStatus').textContent = msg.nightMode ? 'ACTIVE (21:00-10:00)' : 'Inactive';
  document.getElementById('wifiStatus').textContent = msg.wifiConnected ? 'Connected' : (msg.apMode ? 'AP Mode' : 'Disconnected');
  document.getElementById('rtcTime').textContent = msg.rtcTime || '--';
  document.getElementById('fridgeStatus').textContent = msg.fridgeLost ? 'OFFLINE' : 'Online';
  document.getElementById('simCurrentRH').textContent = (typeof msg.hum === 'number') ? msg.hum.toFixed(1) : '--';
  document.getElementById('simBand').textContent = (msg.activeBand != null) ? 'Band ' + msg.activeBand : '--';
  document.getElementById('simConfidence').textContent = (typeof msg.confidence === 'number') ? (msg.confidence * 100).toFixed(1) + '%' : '--';
  document.getElementById('controlMode').textContent = msg.controlMode || '--';

  if (typeof msg.temp === 'number') feedLiveGraph(0, msg.temp);
  if (typeof msg.hum === 'number') feedLiveGraph(1, msg.hum);
  if (typeof msg.co2 === 'number') feedLiveGraph(2, msg.co2);
  if (typeof msg.fridge === 'number') feedLiveGraph(3, msg.fridge);

  // Warmup state handling
  var panel = document.getElementById('warmupPanel');
  if (msg.warmupSelected) {
    if (msg.warmupDuration && msg.warmupDuration > 0) {
      warmupDuration = msg.warmupDuration;
      isWarmupComplete = false;
      if (panel) panel.style.display = 'none';
    } else {
      if (panel) panel.style.display = 'none';
      document.getElementById('warmupBadge').style.display = 'none';
      document.getElementById('warmupProgressContainer').style.display = 'none';
      if (warmupInterval) {
        clearInterval(warmupInterval);
        warmupInterval = null;
      }
      isWarmupComplete = true;
    }
  } else {
    if (panel) panel.style.display = 'block';
    document.getElementById('warmupBadge').style.display = 'none';
    document.getElementById('warmupProgressContainer').style.display = 'none';
    if (warmupInterval) {
      clearInterval(warmupInterval);
      warmupInterval = null;
    }
    isWarmupComplete = false;
  }
}

function updateWarmupUI(remainingSeconds) {
  var badge = document.getElementById('warmupBadge');
  var progressContainer = document.getElementById('warmupProgressContainer');
  var progressBar = document.getElementById('warmupProgressBar');
  var panel = document.getElementById('warmupPanel');

  if (remainingSeconds <= 0 || warmupDuration <= 0) {
    if (panel) panel.style.display = 'none';
    badge.style.display = 'none';
    progressContainer.style.display = 'none';
    if (warmupInterval) {
      clearInterval(warmupInterval);
      warmupInterval = null;
    }
    isWarmupComplete = true;
    return;
  }

  isWarmupComplete = false;
  if (panel) panel.style.display = 'none';
  badge.style.display = 'inline-block';
  progressContainer.style.display = 'block';

  var elapsed = warmupDuration - remainingSeconds;
  var percent = Math.min(100, (elapsed / warmupDuration) * 100);
  progressBar.style.width = percent + '%';

  if (remainingSeconds >= 60) {
    badge.textContent = '⏱️ ' + Math.ceil(remainingSeconds / 60) + 'm remaining';
  } else {
    badge.textContent = '⏱️ ' + Math.ceil(remainingSeconds) + 's remaining';
  }
  badge.style.background = '#58a6ff';
  badge.style.animation = 'pulse-blue 1.5s infinite';
}

function startWarmupClientCountdown(initialRemaining) {
  if (warmupInterval) {
    clearInterval(warmupInterval);
    warmupInterval = null;
  }

  warmupRemaining = initialRemaining;
  updateWarmupUI(warmupRemaining);

  warmupInterval = setInterval(function() {
    warmupRemaining--;
    if (warmupRemaining <= 0) {
      warmupRemaining = 0;
      updateWarmupUI(0);
      if (warmupInterval) {
        clearInterval(warmupInterval);
        warmupInterval = null;
      }
    } else {
      updateWarmupUI(warmupRemaining);
    }
  }, 1000);
}

function updateOverrideStatus(msg){
  var panel = document.getElementById('overridePanel');
  var timeDisplay = document.getElementById('overrideTime');
  var active = msg.humOverride || msg.co2Override;
  var remaining = Math.max(msg.humOverrideRemaining || 0, msg.co2OverrideRemaining || 0);

  if(active && remaining > 0){
    panel.style.display = 'block';
    var min = Math.floor(remaining / 60);
    var sec = remaining % 60;
    timeDisplay.textContent = min + ':' + (sec < 10 ? '0' : '') + sec;
  } else {
    panel.style.display = 'none';
  }

  if (msg.warmupRemaining !== undefined) {
    var newRemaining = Math.max(0, msg.warmupRemaining);
    if (warmupDuration <= 0 && newRemaining > 0) {
      warmupDuration = newRemaining;
    }
    if (warmupDuration > 0 && (!warmupInterval || isWarmupComplete)) {
      startWarmupClientCountdown(newRemaining);
    } else if (warmupDuration > 0) {
      warmupRemaining = newRemaining;
      updateWarmupUI(warmupRemaining);
    }
  }

  if (msg.compressorLocked !== undefined) {
    document.getElementById('compLock').textContent = msg.compressorLocked ? '(COOLDOWN)' : '';
  }
}

function updateRelays(msg){
  var hoh = document.getElementById('hohState');
  hoh.textContent = msg.hoh ? 'ON' : 'OFF';
  hoh.className = 'state ' + (msg.hoh ? 'on' : 'off');
  var hohCard = hoh.parentElement;
  if (msg.hoh) hohCard.classList.add('active'); else hohCard.classList.remove('active');
  hohCard.setAttribute('data-relay-state', msg.hoh ? 'true' : 'false');

  var assist = document.getElementById('assistState');
  assist.textContent = msg.assist ? 'ON' : 'OFF';
  assist.className = 'state ' + (msg.assist ? 'on' : 'off');
  var assistCard = assist.parentElement;
  if (msg.assist) assistCard.classList.add('active'); else assistCard.classList.remove('active');
  assistCard.setAttribute('data-relay-state', msg.assist ? 'true' : 'false');

  var fan = document.getElementById('fanState');
  fan.textContent = msg.fan ? 'ON' : 'OFF';
  fan.className = 'state ' + (msg.fan ? 'on' : 'off');
  var fanCard = fan.parentElement;
  if (msg.fan) fanCard.classList.add('active'); else fanCard.classList.remove('active');
  fanCard.setAttribute('data-relay-state', msg.fan ? 'true' : 'false');

  var comp = document.getElementById('compState');
  comp.textContent = msg.compressor ? 'ON' : 'OFF';
  comp.className = 'state ' + (msg.compressor ? 'on' : 'off');
  var compCard = comp.parentElement;
  if (msg.compressor) compCard.classList.add('active'); else compCard.classList.remove('active');
}

function updateConfig(msg){
  document.getElementById('humHoHFloor').value = msg.humHoHFloor;
  document.getElementById('humAssistFloor').value = msg.humAssistFloor;
  document.getElementById('humCeiling').value = msg.humCeiling;
  document.getElementById('humExhaustOn').value = msg.humExhaustOn;
  document.getElementById('assistOn').value = msg.assistOnSec;
  document.getElementById('assistOff').value = msg.assistOffSec;
  document.getElementById('co2High').value = msg.co2HighLimit;
  document.getElementById('co2Low').value = msg.co2LowTarget;
  document.getElementById('co2Emergency').value = msg.co2Emergency;
  document.getElementById('emaWeight').value = msg.emaWeight;
  document.getElementById('funcPos1').value = msg.funcPos1;
  document.getElementById('funcPos2').value = msg.funcPos2;
  document.getElementById('funcPos3').value = msg.funcPos3;
  document.getElementById('funcPos4').value = msg.funcPos4;
  document.getElementById('weatherLat').value = msg.weatherLat;
  document.getElementById('weatherLon').value = msg.weatherLon;
  document.getElementById('overrideTimeout').value = msg.overrideTimeout || 10;
}

function updateCalibration(msg){
  var countdown = document.getElementById('calibCountdown');
  var status = document.getElementById('calibStatus');
  var startBtn = document.getElementById('calibStartBtn');
  var cancelBtn = document.getElementById('calibCancelBtn');

  if(msg.active){
    var remaining = Math.max(0, msg.remaining);
    var min = Math.floor(remaining / 60);
    var sec = remaining % 60;
    countdown.textContent = min + ':' + (sec < 10 ? '0' : '') + sec;
    status.textContent = 'Calibrating...';
    status.style.color = '#58a6ff';
    startBtn.style.display = 'none';
    cancelBtn.style.display = 'inline-block';
  } else {
    countdown.textContent = '--';
    status.textContent = 'Not active';
    status.style.color = '#8b949e';
    startBtn.style.display = 'inline-block';
    cancelBtn.style.display = 'none';
  }
}

function addLog(message, level){
  var d = new Date();
  var timeStr = d.getHours() + ':' + String(d.getMinutes()).padStart(2,'0') + ':' + String(d.getSeconds()).padStart(2,'0');
  var logArea = document.getElementById('logArea');
  var div = document.createElement('div');
  div.className = 'log-entry ' + (level || 'info');
  div.textContent = '[' + timeStr + '] ' + message;
  logArea.insertBefore(div, logArea.firstChild);
  while (logArea.children.length > 100) {
    logArea.removeChild(logArea.lastChild);
  }
}

function switchTab(element, tabId){
  document.querySelectorAll('.tab').forEach(function(t){ t.classList.remove('active'); });
  document.querySelectorAll('.tab-content').forEach(function(c){ c.classList.remove('active'); });
  element.classList.add('active');
  document.getElementById(tabId).classList.add('active');
}

function startWarmup(seconds) {
  document.querySelectorAll('#warmupPanel button').forEach(function(b) { b.disabled = true; });

  var success = sendWS({type: 6, cmd: 'warmup', duration: seconds});
  var panel = document.getElementById('warmupPanel');

  if (panel) panel.style.display = 'none';

  if (!success) {
    addLog('Failed to send warmup command - WebSocket disconnected', 'warn');
    document.querySelectorAll('#warmupPanel button').forEach(function(b) { b.disabled = false; });
    if (panel) panel.style.display = 'block';
    return;
  }

  if (seconds === 0) {
    addLog('Warmup skipped', 'info');
    return;
  }

  addLog('Warmup started - ' + seconds + ' seconds', 'info');
}

var identifyTimer = null;
var identifyTimeout = null;

function identifyRelay(index) {
  if (identifyTimer) { clearInterval(identifyTimer); identifyTimer = null; }
  if (identifyTimeout) { clearTimeout(identifyTimeout); identifyTimeout = null; }
  var state = false;
  sendWS({type: 6, cmd: 'relay', index: index, state: 1, force: false, confirmed: true, identify: true});
  addLog('Relay identification started - toggling for 5 seconds', 'info');
  identifyTimer = setInterval(function() {
    state = !state;
    sendWS({type: 6, cmd: 'relay', index: index, state: state ? 1 : 0, force: false, confirmed: true, identify: true});
  }, 500);
  identifyTimeout = setTimeout(function() {
    clearInterval(identifyTimer);
    identifyTimer = null;
    identifyTimeout = null;
    sendWS({type: 6, cmd: 'relay', index: index, state: 0, force: false, confirmed: true, identify: true});
    addLog('Relay identification complete', 'info');
  }, 5000);
}

function relayCmd(index, state){
  if(state === 1){
    sendWS({type: 6, cmd: 'relay', index: index, state: state, force: true, confirmed: false});
  } else {
    sendWS({type: 6, cmd: 'relay', index: index, state: state, force: true, confirmed: true});
  }
}

function saveThresholds(){
  var hohFloor = parseFloat(document.getElementById('humHoHFloor').value);
  var assistFloor = parseFloat(document.getElementById('humAssistFloor').value);
  var ceiling = parseFloat(document.getElementById('humCeiling').value);
  var assistOn = parseInt(document.getElementById('assistOn').value, 10);
  var assistOff = parseInt(document.getElementById('assistOff').value, 10);
  var co2High = parseInt(document.getElementById('co2High').value, 10);
  var co2Low = parseInt(document.getElementById('co2Low').value, 10);
  var co2Emer = parseInt(document.getElementById('co2Emergency').value, 10);
  var exhaustOn = parseFloat(document.getElementById('humExhaustOn').value);

  var emaWeight = parseFloat(document.getElementById('emaWeight').value);
  if (isNaN(emaWeight) || emaWeight < 0.10 || emaWeight > 0.50) {
    addLog('EMA Weight must be between 0.10 and 0.50', 'warn');
    return;
  }

  if (isNaN(hohFloor) || isNaN(assistFloor) || isNaN(ceiling) || isNaN(exhaustOn)) {
    addLog('Invalid humidity threshold value', 'warn');
    return;
  }
  if (hohFloor < 0 || hohFloor > 100 || assistFloor < 0 || assistFloor > 100 || ceiling < 0 || ceiling > 100 || exhaustOn < 0 || exhaustOn > 100) {
    addLog('Humidity values must be between 0 and 100', 'warn');
    return;
  }
  if (assistOn < 0 || assistOff < 0) {
    addLog('Assist times cannot be negative', 'warn');
    return;
  }
  if (isNaN(co2High) || isNaN(co2Low) || isNaN(co2Emer)) {
    addLog('Invalid CO2 threshold value', 'warn');
    return;
  }
  if (co2High < 0 || co2Low < 0 || co2Emer < 0) {
    addLog('CO2 values must be positive', 'warn');
    return;
  }

    var wLat = parseFloat(document.getElementById('weatherLat').value);
  var wLon = parseFloat(document.getElementById('weatherLon').value);
  if (isNaN(wLat) || wLat < -90 || wLat > 90 || isNaN(wLon) || wLon < -180 || wLon > 180) {
    addLog('Invalid weather coordinates', 'warn');
    return;
  }

  var overrideTimeout = parseInt(document.getElementById('overrideTimeout').value, 10);
  if (isNaN(overrideTimeout) || overrideTimeout < 1) overrideTimeout = 10;
  if (overrideTimeout > 1440) overrideTimeout = 1440;

   var thresholds = {
    humHoHFloor: hohFloor,
    humAssistFloor: assistFloor,
    humCeiling: ceiling,
    humExhaustOn: exhaustOn,
    assistOnSec: assistOn,
    assistOffSec: assistOff,
    co2HighLimit: co2High,
    co2LowTarget: co2Low,
    co2Emergency: co2Emer,
    emaWeight: emaWeight
  };
  sendWS({type: 6, cmd: 'thresholds_and_ema', data: thresholds});
  addLog('Settings saved!', 'info');
}

function startCalibration(){
  sendWS({type: 6, cmd: 'calibrate_start'});
}

function cancelCalibration(){
  sendWS({type: 6, cmd: 'calibrate_cancel'});
}

function runSimulation(){
  var target = parseFloat(document.getElementById('simTargetRH').value);
  var current = parseFloat(document.getElementById('simCurrentRH').textContent);

  if(isNaN(current)){
    addLog('Waiting for humidity data...', 'warn');
    return;
  }
  if(isNaN(target)){
    addLog('Invalid target RH', 'warn');
    return;
  }

  sendWS({type: 6, cmd: 'simulate', current: current, target: target});
}

function setOverrideTime(minutes, btn) {
  document.getElementById('overrideTimeout').value = minutes;
  document.querySelectorAll('.override-btn').forEach(function(b) { b.classList.remove('active'); });
  btn.classList.add('active');
}

function setRTCTime(){
  var dt = document.getElementById('rtcDateTime').value;
  if(!dt){
    addLog('Please select a date and time', 'warn');
    return;
  }
  sendWS({type: 6, cmd: 'rtc', datetime: dt});
  addLog('RTC time update sent', 'info');
}

function saveRelayMapping(){
  var f1 = parseInt(document.getElementById('funcPos1').value, 10);
  var f2 = parseInt(document.getElementById('funcPos2').value, 10);
  var f3 = parseInt(document.getElementById('funcPos3').value, 10);
  var f4 = parseInt(document.getElementById('funcPos4').value, 10);

  if (new Set([f1, f2, f3, f4]).size !== 4) {
    addLog('Each function must be assigned exactly once!', 'warn');
    return;
  }

  var data = {
    funcPos1: f1,
    funcPos2: f2,
    funcPos3: f3,
    funcPos4: f4
  };
  sendWS({type: 6, cmd: 'relay_mapping', data: data});
  addLog('Relay mapping update sent', 'info');
}

function resumeAutomation(){
  sendWS({type: 6, cmd: 'resume_automation'});
  addLog('Automation resumed', 'info');
}

// ============================================================
// Graph Engine (v1.4)
// ============================================================
var graphChart = null;
var graphSensor = 0;
var graphRange = 86400;
var graphRequestId = 0;
var graphLastRequestTime = 0;
var liveBuffers = [[],[],[],[]];
var GRAPH_MAX_LIVE = 3600;

function initGraph() {
  var canvas = document.getElementById('graphCanvas');
  if (!canvas) return;
  if (typeof Chart === 'undefined') {
    canvas.parentNode.innerHTML = '<p style="color:#d29922;text-align:center;padding:40px;">Chart library not loaded</p>';
    return;
  }
  if (graphChart) { graphChart.destroy(); graphChart = null; }
  liveBuffers = [[],[],[],[]];

  var ctx = canvas.getContext('2d');
  graphChart = new Chart(ctx, {
    type: 'line',
    data: {
      datasets: [
        { label: 'Live', data: [], borderColor: '#58a6ff', borderWidth: 1.5, tension: 0.2, pointRadius: 0 },
        { label: 'History', data: [], borderColor: '#3fb950', borderWidth: 1, tension: 0.2, pointRadius: 0 }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: {
          type: 'linear',
          title: { display: true, text: 'Time', color: '#8b949e' },
          ticks: {
            color: '#8b949e',
            callback: function(v) {
              var d = new Date(v * 1000);
              if (graphRange >= 86400) {
                return (d.getMonth()+1) + '/' + d.getDate() + ' ' +
                       d.getHours() + ':' + String(d.getMinutes()).padStart(2,'0');
              }
              return d.getHours() + ':' + String(d.getMinutes()).padStart(2,'0');
            }
          },
          grid: { color: '#21262d' }
        },
        y: {
          title: { display: true, text: '', color: '#8b949e' },
          ticks: { color: '#8b949e' },
          grid: { color: '#21262d' }
        }
      },
      plugins: {
        legend: { labels: { color: '#8b949e' } }
      }
    }
  });
  graphChart.data.datasets[0].data = liveBuffers[graphSensor];
  updateGraphLabels();
  graphChart.update('none');
}

function updateGraphLabels() {
  var labels = ['Temperature (°C)', 'Humidity (%)', 'CO2 (ppm)', 'Fridge Temp (°C)'];
  if (graphChart && graphChart.options.scales.y) {
    graphChart.options.scales.y.title.text = labels[graphSensor] || '';
  }
}

function switchGraphTab(btn, sensor) {
  graphSensor = sensor;
  document.querySelectorAll('.graph-tab').forEach(function(b){ b.classList.remove('active'); });
  btn.classList.add('active');
  if (graphChart) {
    graphChart.data.datasets[0].data = liveBuffers[sensor];
  }
  updateGraphLabels();
  requestHistorical();
}

function setGraphRange(seconds, btn) {
  graphRange = seconds;
  document.querySelectorAll('.graph-range-btn').forEach(function(b){ b.classList.remove('active'); });
  btn.classList.add('active');
  if (graphChart) { graphChart.update('none'); }
  requestHistorical();
}

function requestHistorical() {
  var now = Date.now();
  if (now - graphLastRequestTime < 5000 && graphSensor === lastRequestedSensor) return;
  graphLastRequestTime = now;
  graphRequestId = (graphRequestId + 1) & 0xFFFF;
  var start = Math.floor(now / 1000) - graphRange;
  sendWS({type: 100, sensor: graphSensor, start: start, end: Math.floor(now / 1000), max: 350, rid: graphRequestId});
  lastRequestedSensor = graphSensor;
}

var lastLiveFeedTime = [0, 0, 0, 0];

function feedLiveGraph(sensor, value) {
  var now = Date.now();
  if (now - lastLiveFeedTime[sensor] < 5000) return;
  lastLiveFeedTime[sensor] = now;

  var epoch = Math.floor(now / 1000);
  liveBuffers[sensor].push({x: epoch, y: value});
  if (liveBuffers[sensor].length > GRAPH_MAX_LIVE) {
    liveBuffers[sensor].shift();
  }
  if (graphChart && sensor === graphSensor) {
    graphChart.data.datasets[0].data = liveBuffers[sensor];
    graphChart.update('none');
  }
}

function handleGraphResponse(msg) {
  if (msg.rid !== graphRequestId || msg.s !== graphSensor) return;
  if (!graphChart) return;
  if (msg.error) { console.warn('Graph error:', msg.error); return; }
  var points = (msg.p || []).map(function(p){ return {x: p[0], y: p[1]}; });
  graphChart.data.datasets[1].data = points;
  graphChart.update('none');
}

// ============================================================
// MASCOTS TAB — DESIGNER ENGINE
// ============================================================

// -------- Templates --------
const DESIGNER_TEMPLATES = {
  amanita: {
    name: 'Amanita',
    grid: [
      [0,0,0,1,1,1,1,1,1,0,0,0],
      [0,0,1,1,1,1,1,1,1,1,0,0],
      [0,1,1,2,1,3,3,1,2,1,1,0],
      [1,1,1,1,1,1,1,1,1,1,1,1],
      [1,1,1,1,1,1,1,1,1,1,1,0],
      [0,1,1,1,1,1,1,1,1,1,0,0],
      [0,0,1,1,1,1,1,1,1,0,0,0],
      [0,0,0,4,4,4,4,4,4,0,0,0],
      [0,0,0,4,4,6,6,4,4,0,0,0],
      [0,0,0,4,7,4,4,7,4,0,0,0],
      [0,0,0,5,5,5,5,5,5,0,0,0],
      [0,0,4,4,4,4,4,4,4,4,0,0],
    ]
  },
  chanterelle: {
    name: 'Chanterelle',
    grid: [
      [0,0,0,1,1,1,1,1,1,0,0,0],
      [0,0,1,1,2,1,1,2,1,1,0,0],
      [0,1,1,1,1,3,3,1,1,1,1,0],
      [1,1,1,1,1,1,1,1,1,1,1,0],
      [1,1,1,1,1,1,1,1,1,1,0,0],
      [0,1,1,1,1,1,1,1,1,0,0,0],
      [0,0,1,1,1,1,1,1,0,0,0,0],
      [0,0,0,4,4,4,4,4,4,0,0,0],
      [0,0,0,4,4,6,6,4,4,0,0,0],
      [0,0,0,4,7,4,4,7,4,0,0,0],
      [0,0,0,5,5,5,5,5,5,0,0,0],
      [0,0,4,4,4,4,4,4,4,4,0,0],
    ]
  },
  shiitake: {
    name: 'Shiitake',
    grid: [
      [0,0,0,1,1,1,1,1,1,0,0,0],
      [0,0,1,1,1,1,1,1,1,1,0,0],
      [0,1,1,2,1,1,1,1,2,1,1,0],
      [1,1,1,1,1,3,3,1,1,1,1,1],
      [1,1,1,1,1,1,1,1,1,1,1,0],
      [0,1,1,1,1,1,1,1,1,1,0,0],
      [0,0,1,1,1,1,1,1,1,0,0,0],
      [0,0,0,4,4,4,4,4,4,0,0,0],
      [0,0,0,4,4,6,6,4,4,0,0,0],
      [0,0,0,4,7,4,4,7,4,0,0,0],
      [0,0,0,5,5,5,5,5,5,0,0,0],
      [0,0,4,4,4,4,4,4,4,4,0,0],
    ]
  },
  magic: {
    name: 'Magic',
    grid: [
      [0,0,0,0,1,1,1,1,0,0,0,0],
      [0,0,0,1,1,2,2,1,1,0,0,0],
      [0,0,1,1,1,3,3,1,1,1,0,0],
      [0,1,1,1,1,1,1,1,1,1,1,0],
      [1,1,1,1,1,1,1,1,1,1,1,0],
      [0,1,1,1,1,1,1,1,1,1,0,0],
      [0,0,1,1,1,1,1,1,1,0,0,0],
      [0,0,0,4,4,4,4,4,4,0,0,0],
      [0,0,0,4,4,6,6,4,4,0,0,0],
      [0,0,0,4,7,4,4,7,4,0,0,0],
      [0,0,0,5,5,5,5,5,5,0,0,0],
      [0,0,4,4,4,4,4,4,4,4,0,0],
    ]
  },
  morel: {
    name: 'Morel',
    grid: [
      [0,0,0,1,1,1,1,1,1,0,0,0],
      [0,0,1,1,2,1,1,2,1,1,0,0],
      [0,1,1,1,1,1,1,1,1,1,1,0],
      [1,1,1,1,3,1,1,3,1,1,1,1],
      [1,1,1,1,1,1,1,1,1,1,1,0],
      [0,1,1,1,1,1,1,1,1,1,0,0],
      [0,0,1,1,1,1,1,1,1,0,0,0],
      [0,0,0,4,4,4,4,4,4,0,0,0],
      [0,0,0,4,4,6,6,4,4,0,0,0],
      [0,0,0,4,7,4,4,7,4,0,0,0],
      [0,0,0,5,5,5,5,5,5,0,0,0],
      [0,0,4,4,4,4,4,4,4,4,0,0],
    ]
  }
};

const SPOT_COORDS = {
  amanita: [{r:2,c:4}, {r:2,c:7}, {r:4,c:3}, {r:4,c:8}, {r:1,c:5}, {r:1,c:6}],
  chanterelle: [{r:2,c:5}, {r:2,c:8}, {r:4,c:3}, {r:4,c:9}, {r:1,c:4}, {r:5,c:6}],
  shiitake: [{r:2,c:5}, {r:2,c:8}, {r:4,c:3}, {r:4,c:9}, {r:1,c:4}, {r:5,c:6}],
  magic: [{r:2,c:4}, {r:2,c:7}, {r:4,c:3}, {r:4,c:8}, {r:1,c:5}, {r:1,c:6}],
  morel: [{r:2,c:5}, {r:2,c:8}, {r:4,c:3}, {r:4,c:9}, {r:1,c:4}, {r:5,c:6}]
};

const designerState = {
  template: 'amanita',
  capWidth: 100,
  capHeight: 100,
  capColor: '#e63946',
  stemHeight: 100,
  stemWidth: 100,
  stemColor: '#fefae0',
  spots: 2,
  bounceHeight: 20,
  animSpeed: 22,
  // Face customization
  eyeStyle: 'normal',
  mouthStyle: 'smile',
  blushEnabled: true,
  blushColor: '#ff99c8',
  blushSize: 1.0,
  capTexture: 'smooth'
};

function lightenColor(hex, percent) {
  const num = parseInt(hex.replace('#', ''), 16);
  const r = Math.min(255, (num >> 16) + percent);
  const g = Math.min(255, ((num >> 8) & 0x00FF) + percent);
  const b = Math.min(255, (num & 0x0000FF) + percent);
  return '#' + (1 << 24 | r << 16 | g << 8 | b).toString(16).slice(1);
}

// ─── EYE STYLES ───
function getEyePixels(style) {
  switch(style) {
    case 'normal': return [{dr:0, dc:0, w:1, h:1, r:1}];
    case 'happy':  return [{dr:0, dc:0, w:1, h:1, r:1}, {dr:0, dc:0, w:1, h:1, r:1}];
    case 'sleepy': return [{dr:0, dc:0, w:2, h:0.5, r:0}];
    case 'closed': return [{dr:0, dc:0, w:2, h:0.3, r:0}];
    case 'big':    return [{dr:0, dc:0, w:1.5, h:1.5, r:2}];
    default:       return [];
  }
}

// ─── MOUTH STYLES ───
function getMouthPixels(style) {
  switch(style) {
    case 'smile':    return [{dr:0, dc:0, w:2, h:1, r:0}];
    case 'happy':    return [{dr:0, dc:0, w:3, h:1.5, r:2}];
    case 'neutral':  return [{dr:0, dc:0, w:1.5, h:0.3, r:0}];
    case 'surprised':return [{dr:0, dc:0, w:1.5, h:1.5, r:2}];
    default:         return [];
  }
}

function darkenColor(hex, percent) {
  const num = parseInt(hex.replace('#', ''), 16);
  const r = Math.max(0, (num >> 16) - percent);
  const g = Math.max(0, ((num >> 8) & 0x00FF) - percent);
  const b = Math.max(0, (num & 0x0000FF) - percent);
  return '#' + (1 << 24 | r << 16 | g << 8 | b).toString(16).slice(1);
}

function isHexColor(str) {
  return /^#[0-9a-fA-F]{6}$/.test(str);
}

// ─── FACE RENDERER ───
function renderFace(sprite, grid, cols, pixelSize, state) {
  const faceRow = 8;
  const leftEyeCol = 5;
  const rightEyeCol = 7;
  const mouthRow = 9;
  const blushRow = 9;
  const leftBlushCol = 4;
  const rightBlushCol = 8;

  const eyeColor = '#1a1a1a';
  const mouthColor = '#1a1a1a';
  const blushColor = state.blushColor || '#ff99c8';
  const blushSize = state.blushSize || 1.0;

  // Eyes
  if (state.eyeStyle !== 'none') {
    const eyePixels = getEyePixels(state.eyeStyle);
    eyePixels.forEach(function(p) {
      const r = faceRow + p.dr;
      const c = leftEyeCol + p.dc;
      const px = document.createElement('div');
      px.className = 'pixel';
      px.style.cssText = 'position:absolute;width:' + (pixelSize * (p.w||1)) + 'px;height:' + (pixelSize * (p.h||1)) + 'px;left:' + (c * pixelSize) + 'px;top:' + (r * pixelSize) + 'px;background-color:' + eyeColor + ';border-radius:' + (p.r||1) + 'px;';
      sprite.appendChild(px);
    });
    eyePixels.forEach(function(p) {
      const r = faceRow + p.dr;
      const c = rightEyeCol + (-p.dc);
      const px = document.createElement('div');
      px.className = 'pixel';
      px.style.cssText = 'position:absolute;width:' + (pixelSize * (p.w||1)) + 'px;height:' + (pixelSize * (p.h||1)) + 'px;left:' + (c * pixelSize) + 'px;top:' + (r * pixelSize) + 'px;background-color:' + eyeColor + ';border-radius:' + (p.r||1) + 'px;';
      sprite.appendChild(px);
    });
  }

  // Mouth
  if (state.mouthStyle !== 'none') {
    const mouthPixels = getMouthPixels(state.mouthStyle);
    mouthPixels.forEach(function(p) {
      const r = mouthRow + p.dr;
      const c = leftEyeCol + p.dc + 1;
      const px = document.createElement('div');
      px.className = 'pixel';
      px.style.cssText = 'position:absolute;width:' + (pixelSize * (p.w||1)) + 'px;height:' + (pixelSize * (p.h||1)) + 'px;left:' + (c * pixelSize) + 'px;top:' + (r * pixelSize) + 'px;background-color:' + mouthColor + ';border-radius:' + (p.r||1) + 'px;';
      sprite.appendChild(px);
    });
  }

  // Blush
  if (state.blushEnabled) {
    const blushSizePx = Math.max(1, Math.round(blushSize * pixelSize));
    const blushPixels = [
      { r: blushRow, c: leftBlushCol },
      { r: blushRow + 1, c: leftBlushCol },
      { r: blushRow, c: rightBlushCol },
      { r: blushRow + 1, c: rightBlushCol }
    ];
    blushPixels.forEach(function(p) {
      const px = document.createElement('div');
      px.className = 'pixel';
      px.style.cssText = 'position:absolute;width:' + blushSizePx + 'px;height:' + blushSizePx + 'px;left:' + (p.c * pixelSize) + 'px;top:' + (p.r * pixelSize) + 'px;background-color:' + blushColor + ';border-radius:50%;opacity:0.7;';
      sprite.appendChild(px);
    });
  }
}

function renderDesignerSprite() {
  const template = DESIGNER_TEMPLATES[designerState.template];
  if (!template) return;
  const sprite = document.getElementById('designerSprite');
  if (!sprite) return;
  const grid = template.grid;
  const rows = grid.length;
  const cols = grid[0].length;
  sprite.innerHTML = '';

  const capScaleX = designerState.capWidth / 100;
  const capScaleY = designerState.capHeight / 100;
  const stemScaleY = designerState.stemHeight / 100;
  const stemScaleX = designerState.stemWidth / 100;
  const capRows = 7;
  const stemRows = rows - capRows;

  const capColor = designerState.capColor;
  const stemColor = designerState.stemColor;
  const spotColor = '#ffffff';

  const colorMap = {
    1: capColor,
    2: lightenColor(capColor, 30),
    3: spotColor,
    4: stemColor,
    5: darkenColor(stemColor, 20),
    6: '#1a1a1a',
    7: '#ff99c8'
  };

  const pixelSize = 5;
  const spots = SPOT_COORDS[designerState.template] || [];
  const spotsToShow = designerState.spots;

  // Draw base mushroom, skip face pixels (6,7)
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      let val = grid[r][c];
      // Skip face pixels — we draw them separately
      if (val === 6 || val === 7) continue;

      let isSpot = false;
      for (let i = 0; i < spotsToShow && i < spots.length; i++) {
        if (spots[i].r === r && spots[i].c === c) { isSpot = true; break; }
      }
      if (isSpot) val = 3;
      if (val === 0) continue;

      let scaleX = 1, scaleY = 1;
      let offsetX = 0, offsetY = 0;
      if (r < capRows) {
        scaleX = capScaleX; scaleY = capScaleY;
        offsetX = (cols * pixelSize * (1 - capScaleX)) / 2;
        offsetY = (capRows * pixelSize * (1 - capScaleY)) / 2;
      } else {
        scaleX = stemScaleX; scaleY = stemScaleY;
        offsetX = (cols * pixelSize * (1 - stemScaleX)) / 2;
        offsetY = (stemRows * pixelSize * (1 - stemScaleY)) / 2;
      }

      const px = (c * pixelSize * scaleX) + offsetX;
      const py = (r * pixelSize * scaleY) + offsetY;

      const pixel = document.createElement('div');
      pixel.className = 'pixel';
      pixel.style.left = px + 'px';
      pixel.style.top = py + 'px';
      pixel.style.width = (pixelSize * scaleX) + 'px';
      pixel.style.height = (pixelSize * scaleY) + 'px';
      pixel.style.borderRadius = '1px';
      pixel.style.backgroundColor = colorMap[val] || '#ff00ff';
      sprite.appendChild(pixel);
    }
  }

  // ─── Draw face overlay ───
  renderFace(sprite, grid, cols, pixelSize, designerState);

  const speedSec = designerState.animSpeed / 10;
  sprite.style.animation = 'designerBounce ' + speedSec + 's ease-in-out infinite';
  document.getElementById('designerName').textContent = template.name;
}

// ============================================================
// MASCOTS TAB — PROFILE MANAGEMENT
// ============================================================

var profileList = [];

function loadProfileList() {
  sendWS({type: 6, cmd: 'profile_list'});
}

function updateProfileDropdown() {
  const select = document.getElementById('profileLoadSelect');
  if (!select) return;
  const currentVal = select.value;
  select.innerHTML = '';
  if (profileList.length === 0) {
    select.innerHTML = '<option value="">-- No saved profiles --</option>';
  } else {
    profileList.forEach(function(name) {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      select.appendChild(opt);
    });
  }
  if (currentVal && profileList.indexOf(currentVal) !== -1) {
    select.value = currentVal;
  }
  const countEl = document.getElementById('profileCount');
  if (countEl) {
    countEl.textContent = '(' + profileList.length + '/8 profiles used)';
  }
  const saveBtn = document.getElementById('saveProfileBtn');
  if (saveBtn) {
    const nameInput = document.getElementById('profileNameInput');
    const name = nameInput ? nameInput.value.trim() : '';
    saveBtn.disabled = (profileList.length >= 8 && profileList.indexOf(name) === -1);
  }
}

function saveCurrentProfile() {
  const nameInput = document.getElementById('profileNameInput');
  const name = nameInput.value.trim();
  if (!name) { addLog('Please enter a profile name', 'warn'); return; }
  if (profileList.length >= 8 && profileList.indexOf(name) === -1) {
    addLog('Maximum 8 profiles reached', 'warn'); return;
  }

  const template = DESIGNER_TEMPLATES[designerState.template];
  const data = {
    version: 1,
    template: designerState.template,
    capWidth: designerState.capWidth,
    capHeight: designerState.capHeight,
    capColor: designerState.capColor,
    stemHeight: designerState.stemHeight,
    stemWidth: designerState.stemWidth,
    stemColor: designerState.stemColor,
    spots: designerState.spots,
    bounceHeight: designerState.bounceHeight,
    animSpeed: designerState.animSpeed,
    // Face settings
    eyeStyle: designerState.eyeStyle,
    mouthStyle: designerState.mouthStyle,
    blushEnabled: designerState.blushEnabled,
    blushColor: designerState.blushColor,
    blushSize: designerState.blushSize,
    capTexture: designerState.capTexture
  };

  const saveBtn = document.getElementById('saveProfileBtn');
  saveBtn.disabled = true;
  saveBtn.textContent = '💾 Saving...';
  sendWS({type: 6, cmd: 'profile_save', name: name, data: data});
}

function loadSelectedProfile() {
  const select = document.getElementById('profileLoadSelect');
  const name = select.value;
  if (!name) { addLog('No profile selected', 'warn'); return; }
  sendWS({type: 6, cmd: 'profile_load', name: name});
  addLog('Loading profile: ' + name, 'info');
}

function deleteSelectedProfile() {
  const select = document.getElementById('profileLoadSelect');
  const name = select.value;
  if (!name) { addLog('No profile selected', 'warn'); return; }
  if (!confirm('Delete profile "' + name + '"?')) return;
  const deleteBtn = document.getElementById('deleteProfileBtn');
  deleteBtn.disabled = true;
  deleteBtn.textContent = '🗑 Deleting...';
  sendWS({type: 6, cmd: 'profile_delete', name: name});
}

function exportDesignerJSON() {
  const template = DESIGNER_TEMPLATES[designerState.template];
  const exportData = {
    name: template.name,
    template: designerState.template,
    capColor: designerState.capColor,
    stemColor: designerState.stemColor,
    capWidth: designerState.capWidth,
    capHeight: designerState.capHeight,
    stemHeight: designerState.stemHeight,
    stemWidth: designerState.stemWidth,
    spots: designerState.spots,
    bounceHeight: designerState.bounceHeight,
    animSpeed: designerState.animSpeed / 10,
    grid: template.grid,
    // Face settings
    eyeStyle: designerState.eyeStyle,
    mouthStyle: designerState.mouthStyle,
    blushEnabled: designerState.blushEnabled,
    blushColor: designerState.blushColor,
    blushSize: designerState.blushSize,
    capTexture: designerState.capTexture
  };
  const json = JSON.stringify(exportData, null, 2);
  const area = document.getElementById('exportArea');
  area.textContent = json;
  area.classList.add('show');
  const range = document.createRange();
  range.selectNodeContents(area);
  const sel = window.getSelection();
  sel.removeAllRanges();
  sel.addRange(range);
  if (navigator.clipboard) {
    navigator.clipboard.writeText(json).then(function() {
      addLog('JSON copied to clipboard!', 'info');
    }).catch(function() {});
  }
}

var importVisible = false;

function toggleImport() {
  const area = document.getElementById('importArea');
  importVisible = !importVisible;
  area.classList.toggle('show', importVisible);
  if (importVisible) area.focus();
}

function importDesignerJSON() {
  const area = document.getElementById('importArea');
  const text = area.value.trim();
  if (!text) { addLog('Paste JSON to import', 'warn'); return; }
  try {
    const data = JSON.parse(text);
    if (!data.template || !DESIGNER_TEMPLATES[data.template]) {
      addLog('Invalid template in JSON', 'warn'); return;
    }
    if (!data.capColor || !isHexColor(data.capColor)) {
      addLog('Invalid capColor in JSON', 'warn'); return;
    }
    if (!data.stemColor || !isHexColor(data.stemColor)) {
      addLog('Invalid stemColor in JSON', 'warn'); return;
    }

    designerState.template = data.template || 'amanita';
    designerState.capWidth = data.capWidth || 100;
    designerState.capHeight = data.capHeight || 100;
    designerState.capColor = data.capColor || '#e63946';
    designerState.stemHeight = data.stemHeight || 100;
    designerState.stemWidth = data.stemWidth || 100;
    designerState.stemColor = data.stemColor || '#fefae0';
    designerState.spots = (data.spots !== undefined && data.spots !== null) ? data.spots : 2;
    designerState.bounceHeight = data.bounceHeight || 20;
    designerState.animSpeed = data.animSpeed ? data.animSpeed * 10 : 22;
    // Face settings
    designerState.eyeStyle = data.eyeStyle || 'normal';
    designerState.mouthStyle = data.mouthStyle || 'smile';
    designerState.blushEnabled = data.blushEnabled !== undefined ? data.blushEnabled : true;
    designerState.blushColor = data.blushColor || '#ff99c8';
    designerState.blushSize = data.blushSize || 1.0;
    designerState.capTexture = data.capTexture || 'smooth';

    if (data.grid && Array.isArray(data.grid) && data.grid.length === 12) {
      DESIGNER_TEMPLATES[designerState.template].grid = data.grid;
    }

    updateDesignerUI();
    addLog('JSON imported successfully!', 'info');
    area.value = '';
    area.classList.remove('show');
    importVisible = false;
  } catch (e) {
    addLog('Invalid JSON: ' + e.message, 'warn');
  }
}

function randomiseDesigner() {
  const templates = ['amanita', 'chanterelle', 'shiitake', 'magic', 'morel'];
  const capColors = ['#e63946', '#f4a261', '#795548', '#7b2cbf', '#40916c', '#ff6b6b', '#ffd93d', '#6bcbff', '#a66cff'];
  const stemColors = ['#fefae0', '#f5e6d0', '#efebe9', '#e8d5b8', '#f0e6d3', '#e0d5c0'];

  designerState.template = templates[Math.floor(Math.random() * templates.length)];
  designerState.capColor = capColors[Math.floor(Math.random() * capColors.length)];
  designerState.stemColor = stemColors[Math.floor(Math.random() * stemColors.length)];
  designerState.capWidth = 70 + Math.floor(Math.random() * 50);
  designerState.capHeight = 70 + Math.floor(Math.random() * 50);
  designerState.stemHeight = 70 + Math.floor(Math.random() * 60);
  designerState.stemWidth = 60 + Math.floor(Math.random() * 50);
  designerState.bounceHeight = 10 + Math.floor(Math.random() * 25);
  designerState.animSpeed = 15 + Math.floor(Math.random() * 18);
  designerState.spots = [0, 2, 4, 6][Math.floor(Math.random() * 4)];

  updateDesignerUI();
  addLog('Randomised!', 'info');
}

function resetDesigner() {
  designerState.template = 'amanita';
  designerState.capWidth = 100;
  designerState.capHeight = 100;
  designerState.capColor = '#e63946';
  designerState.stemHeight = 100;
  designerState.stemWidth = 100;
  designerState.stemColor = '#fefae0';
  designerState.spots = 2;
  designerState.bounceHeight = 20;
  designerState.animSpeed = 22;
  updateDesignerUI();
  addLog('Reset to default', 'info');
}

function updateDesignerUI() {
  document.getElementById('capWidth').value = designerState.capWidth;
  document.getElementById('capHeight').value = designerState.capHeight;
  document.getElementById('stemHeight').value = designerState.stemHeight;
  document.getElementById('stemWidth').value = designerState.stemWidth;
  document.getElementById('bounceHeight').value = designerState.bounceHeight;
  document.getElementById('animSpeed').value = designerState.animSpeed;

  document.getElementById('capWidthVal').textContent = designerState.capWidth + '%';
  document.getElementById('capHeightVal').textContent = designerState.capHeight + '%';
  document.getElementById('stemHeightVal').textContent = designerState.stemHeight + '%';
  document.getElementById('stemWidthVal').textContent = designerState.stemWidth + '%';
  document.getElementById('bounceHeightVal').textContent = designerState.bounceHeight + 'px';
  document.getElementById('animSpeedVal').textContent = (designerState.animSpeed / 10) + 's';

  document.querySelectorAll('#templateButtons button').forEach(function(b) {
    b.classList.toggle('active', b.dataset.template === designerState.template);
  });
  document.querySelectorAll('[data-spots]').forEach(function(b) {
    b.classList.toggle('active', parseInt(b.dataset.spots) === designerState.spots);
  });
  document.querySelectorAll('#capColorPicker .color-swatch[data-color]').forEach(function(s) {
    s.classList.toggle('active', s.dataset.color === designerState.capColor);
  });
  document.querySelectorAll('#stemColorPicker .color-swatch[data-color]').forEach(function(s) {
    s.classList.toggle('active', s.dataset.color === designerState.stemColor);
  });
  document.getElementById('customCapColor').value = designerState.capColor;
  document.getElementById('customStemColor').value = designerState.stemColor;

  // ─── Face Controls ───
  document.querySelectorAll('[data-eye]').forEach(function(b) {
    b.classList.toggle('active', b.dataset.eye === designerState.eyeStyle);
  });
  document.querySelectorAll('[data-mouth]').forEach(function(b) {
    b.classList.toggle('active', b.dataset.mouth === designerState.mouthStyle);
  });
  document.getElementById('blushEnable').checked = designerState.blushEnabled;
  document.getElementById('blushColor').value = designerState.blushColor;
  document.getElementById('blushSize').value = designerState.blushSize * 10;
  document.getElementById('blushSizeVal').textContent = designerState.blushSize.toFixed(1);

  document.querySelectorAll('[data-texture]').forEach(function(b) {
    b.classList.toggle('active', b.dataset.texture === designerState.capTexture);
  });

  renderDesignerSprite();
}

// ============================================================
// MASCOTS TAB — PROFILE RESPONSE HANDLER
// ============================================================

function handleProfileResponse(msg) {
  if (msg.cmd === 'profile_list') {
    profileList = msg.names || [];
    updateProfileDropdown();
    // Render background stars after profiles load
   } else if (msg.cmd === 'profile_load') {
    if (msg.data) {
      const data = msg.data;
      designerState.template = data.template || 'amanita';
      designerState.capWidth = data.capWidth || 100;
      designerState.capHeight = data.capHeight || 100;
      designerState.capColor = data.capColor || '#e63946';
      designerState.stemHeight = data.stemHeight || 100;
      designerState.stemWidth = data.stemWidth || 100;
      designerState.stemColor = data.stemColor || '#fefae0';
      designerState.spots = (data.spots !== undefined && data.spots !== null) ? data.spots : 2;
      designerState.bounceHeight = data.bounceHeight || 20;
      designerState.animSpeed = data.animSpeed || 22;
      // Face settings
      designerState.eyeStyle = data.eyeStyle || 'normal';
      designerState.mouthStyle = data.mouthStyle || 'smile';
      designerState.blushEnabled = data.blushEnabled !== undefined ? data.blushEnabled : true;
      designerState.blushColor = data.blushColor || '#ff99c8';
      designerState.blushSize = data.blushSize || 1.0;
      designerState.capTexture = data.capTexture || 'smooth';
      updateDesignerUI();
      addLog('Profile loaded: ' + msg.name, 'info');
      document.getElementById('profileNameInput').value = msg.name;
    }
  } else if (msg.cmd === 'profile_save') {
    const saveBtn = document.getElementById('saveProfileBtn');
    saveBtn.disabled = false;
    saveBtn.textContent = '💾 Save';
    if (msg.status === 'ok') {
      addLog('Profile saved successfully', 'info');
      loadProfileList();
      document.getElementById('profileNameInput').value = '';
    } else {
      addLog('Save failed: ' + (msg.message || 'unknown error'), 'warn');
    }
  } else if (msg.cmd === 'profile_delete') {
    const deleteBtn = document.getElementById('deleteProfileBtn');
    deleteBtn.disabled = false;
    deleteBtn.textContent = '🗑 Delete';
    if (msg.status === 'ok') {
      addLog('Profile deleted', 'info');
      loadProfileList();
    } else {
      addLog('Delete failed: ' + (msg.message || 'unknown error'), 'warn');
    }
  } else if (msg.status === 'error') {
    addLog('Profile error: ' + (msg.message || 'unknown error'), 'warn');
    const saveBtn = document.getElementById('saveProfileBtn');
    if (saveBtn) { saveBtn.disabled = false; saveBtn.textContent = '💾 Save'; }
    const deleteBtn = document.getElementById('deleteProfileBtn');
    if (deleteBtn) { deleteBtn.disabled = false; deleteBtn.textContent = '🗑 Delete'; }
  }
}

// ============================================================
// DESIGNER INIT
// ============================================================

function initDesigner() {
  if (!document.getElementById('mascots')) return;

  document.querySelectorAll('#templateButtons button').forEach(function(btn) {
    btn.addEventListener('click', function() {
      designerState.template = this.dataset.template;
      updateDesignerUI();
    });
  });

  ['capWidth', 'capHeight', 'stemHeight', 'stemWidth', 'bounceHeight', 'animSpeed'].forEach(function(id) {
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener('input', function() {
      const val = parseInt(this.value);
      designerState[id] = val;
      const label = document.getElementById(id + 'Val');
      if (label) {
        if (id === 'bounceHeight') label.textContent = val + 'px';
        else if (id === 'animSpeed') label.textContent = (val / 10) + 's';
        else label.textContent = val + '%';
      }
      renderDesignerSprite();
    });
  });

  document.querySelectorAll('#capColorPicker .color-swatch[data-color]').forEach(function(sw) {
    sw.addEventListener('click', function() {
      designerState.capColor = this.dataset.color;
      updateDesignerUI();
    });
  });
  document.getElementById('customCapColor').addEventListener('input', function() {
    designerState.capColor = this.value;
    updateDesignerUI();
  });

  document.querySelectorAll('#stemColorPicker .color-swatch[data-color]').forEach(function(sw) {
    sw.addEventListener('click', function() {
      designerState.stemColor = this.dataset.color;
      updateDesignerUI();
    });
  });
  document.getElementById('customStemColor').addEventListener('input', function() {
    designerState.stemColor = this.value;
    updateDesignerUI();
  });

  document.querySelectorAll('[data-spots]').forEach(function(btn) {
    btn.addEventListener('click', function() {
      designerState.spots = parseInt(this.dataset.spots);
      updateDesignerUI();
    });
  });

  document.getElementById('saveProfileBtn').addEventListener('click', saveCurrentProfile);
  document.getElementById('loadProfileBtn').addEventListener('click', loadSelectedProfile);
  document.getElementById('deleteProfileBtn').addEventListener('click', deleteSelectedProfile);

  document.getElementById('profileNameInput').addEventListener('keydown', function(e) {
    if (e.key === 'Enter') saveCurrentProfile();
  });
  document.getElementById('profileNameInput').addEventListener('input', function() {
    updateProfileDropdown();
  });

  document.getElementById('randomizeBtn').addEventListener('click', randomiseDesigner);
  document.getElementById('resetBtn').addEventListener('click', resetDesigner);
  document.getElementById('exportBtn').addEventListener('click', exportDesignerJSON);

  document.getElementById('importToggleBtn').addEventListener('click', toggleImport);
  document.getElementById('importArea').addEventListener('keydown', function(e) {
    if (e.key === 'Enter' && e.ctrlKey) { importDesignerJSON(); }
  });
  const importBtn = document.createElement('button');
  importBtn.textContent = '📥 Import';
  importBtn.className = 'btn-import';
  importBtn.style.cssText = 'background:#30363d;color:#c9d1d9;border:none;border-radius:6px;padding:6px 14px;cursor:pointer;font-size:0.85em;margin-top:6px;';
  importBtn.addEventListener('click', importDesignerJSON);
  document.getElementById('importArea').parentNode.appendChild(importBtn);

  // ─── Face Controls ───
  // Eyes
  document.querySelectorAll('[data-eye]').forEach(function(btn) {
    btn.addEventListener('click', function() {
      designerState.eyeStyle = this.dataset.eye;
      updateDesignerUI();
    });
  });

  // Mouth
  document.querySelectorAll('[data-mouth]').forEach(function(btn) {
    btn.addEventListener('click', function() {
      designerState.mouthStyle = this.dataset.mouth;
      updateDesignerUI();
    });
  });

  // Blush
  document.getElementById('blushEnable').addEventListener('change', function() {
    designerState.blushEnabled = this.checked;
    updateDesignerUI();
  });
  document.getElementById('blushColor').addEventListener('input', function() {
    designerState.blushColor = this.value;
    updateDesignerUI();
  });
  document.getElementById('blushSize').addEventListener('input', function() {
    designerState.blushSize = parseInt(this.value) / 10;
    document.getElementById('blushSizeVal').textContent = designerState.blushSize.toFixed(1);
    updateDesignerUI();
  });

  // Cap Texture
  document.querySelectorAll('[data-texture]').forEach(function(btn) {
    btn.addEventListener('click', function() {
      designerState.capTexture = this.dataset.texture;
      updateDesignerUI();
    });
  });

  updateDesignerUI();
  loadProfileList();

  // Initialize stars
  initBackgroundStars();
}
// ============================================================
// BACKGROUND STARS ENGINE
// ============================================================

// -------- Settings --------
const STAR_SETTINGS_VERSION = 1;

let starSettings = {
  version: STAR_SETTINGS_VERSION,
  enabled: true,
  size: 56,
  opacity: 22,
  driftSpeed: 22,
  spinRPM: 6,
  pulseSpeed: 4.0,
  driftRange: 200
};

function loadStarSettings() {
  try {
    const saved = localStorage.getItem('growhub32-starSettings');
    if (saved) {
      const parsed = JSON.parse(saved);
      if (parsed.version === STAR_SETTINGS_VERSION) {
        Object.assign(starSettings, parsed);
      } else {
        // Migrate old settings
        starSettings.enabled = parsed.enabled !== undefined ? parsed.enabled : true;
        starSettings.size = parsed.size || 56;
        starSettings.opacity = parsed.opacity || 22;
        starSettings.driftSpeed = parsed.driftSpeed || 22;
        starSettings.spinRPM = parsed.spinRPM || 6;
        starSettings.pulseSpeed = parsed.pulseSpeed || 4.0;
        starSettings.driftRange = parsed.driftRange || 200;
        starSettings.version = STAR_SETTINGS_VERSION;
      }
    }
  } catch(e) { /* ignore */ }
}

function saveStarSettings() {
  try {
    localStorage.setItem('growhub32-starSettings', JSON.stringify(starSettings));
  } catch(e) { /* ignore */ }
}

// -------- Helpers --------
function getMaxStars() {
  if (window.innerWidth < 480) return 2;
  if (window.innerWidth < 768) return 4;
  return 6;
}

function generateDriftPath(range, speed) {
  const r = range || 200;
  const duration = (speed || 22) + (Math.random() - 0.5) * 8;
  return {
    duration: Math.max(12, duration),
    waypoints: [
      { x: (Math.random() - 0.5) * r * 1.2, y: (Math.random() - 0.5) * r * 0.8 },
      { x: (Math.random() - 0.5) * r * 0.7, y: (Math.random() - 0.5) * r * 1.3 },
      { x: (Math.random() - 0.5) * r * 0.9, y: (Math.random() - 0.5) * r * 1.1 },
      { x: (Math.random() - 0.5) * r * 0.6, y: (Math.random() - 0.5) * r * 0.9 },
      { x: (Math.random() - 0.5) * r * 1.1, y: (Math.random() - 0.5) * r * 0.7 }
    ]
  };
}

// -------- Per-star Keyframe Injection --------
function injectDriftKeyframes(index, waypoints, duration) {
  const styleId = 'star-drift-kf-' + index;
  const old = document.getElementById(styleId);
  if (old) old.remove();

  const style = document.createElement('style');
  style.id = styleId;
  style.textContent =
    '@keyframes starDrift-' + index + ' {\n' +
    '  0%   { transform: translate(0px, 0px); }\n' +
    '  20%  { transform: translate(' + waypoints[0].x + 'px, ' + waypoints[0].y + 'px); }\n' +
    '  40%  { transform: translate(' + waypoints[1].x + 'px, ' + waypoints[1].y + 'px); }\n' +
    '  60%  { transform: translate(' + waypoints[2].x + 'px, ' + waypoints[2].y + 'px); }\n' +
    '  80%  { transform: translate(' + waypoints[3].x + 'px, ' + waypoints[3].y + 'px); }\n' +
    '  100% { transform: translate(' + waypoints[4].x + 'px, ' + waypoints[4].y + 'px); }\n' +
    '}';
  document.head.appendChild(style);
  return 'starDrift-' + index;
}

// -------- Create a Single Star --------
function createStar(profile, settings, index) {
  const template = DESIGNER_TEMPLATES[profile.template || 'amanita'];
  if (!template) return null;

  const xPos = 5 + Math.random() * 90;
  const yPos = 10 + Math.random() * 80;

  const size = settings.size + (Math.random() - 0.5) * 12;
  const opacity = (settings.opacity / 100) + (Math.random() - 0.5) * 0.04;

  const drift = generateDriftPath(settings.driftRange, settings.driftSpeed);
  const spinDuration = 60 / (settings.spinRPM / 10);
  const spinDirection = Math.random() > 0.5 ? 'normal' : 'reverse';

  // Inject per-star keyframe
  const kfName = injectDriftKeyframes(index, drift.waypoints, drift.duration);

  // Build wrapper
  const wrapper = document.createElement('div');
  wrapper.className = 'star-drift-wrapper';
  wrapper.style.cssText =
    'left:' + xPos + '%;top:' + yPos + '%;' +
    'width:' + size + 'px;height:' + (size * 1.2) + 'px;' +
    '--star-drift-name:' + kfName + ';' +
    '--drift-duration:' + drift.duration + 's;';

  // Spin wrapper
  const spinWrapper = document.createElement('div');
  spinWrapper.className = 'star-spin-wrapper';
  spinWrapper.style.cssText =
    'width:100%;height:100%;' +
    '--spin-duration:' + spinDuration + 's;' +
    '--spin-direction:' + spinDirection + ';';

  // Pulse layer
  const pulseLayer = document.createElement('div');
  pulseLayer.className = 'star-pulse-layer';
  pulseLayer.style.cssText =
    'width:100%;height:100%;' +
    '--pulse-duration:' + settings.pulseSpeed + 's;';

  // Sprite with box-shadow
  const sprite = document.createElement('div');
  sprite.className = 'star-sprite';
  sprite.style.cssText =
    'width:100%;height:100%;position:relative;' +
    '--star-opacity:' + Math.max(0.08, Math.min(0.45, opacity)) + ';' +
    'image-rendering:pixelated;';

  const grid = template.grid;
  const cols = grid[0].length;
  const rows = grid.length;
  const pixelSize = Math.max(1, size / cols);

  // Color mapping
  const colorMap = {
    1: profile.capColor || '#e63946',
    2: lightenColor(profile.capColor || '#e63946', 30),
    3: '#ffffff',
    4: profile.stemColor || '#fefae0',
    5: darkenColor(profile.stemColor || '#fefae0', 20),
    6: '#1a1a1a',
    7: '#ff99c8'
  };

  let shadows = [];
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const val = grid[r][c];
      if (val === 0) continue;
      if (val === 3 && pixelSize < 1.5) continue;
      const color = colorMap[val] || '#ff00ff';
      shadows.push(c + 'px ' + r + 'px 0 0 ' + color);
    }
  }

  const pixelDiv = document.createElement('div');
  pixelDiv.className = 'pixel-grid';
  pixelDiv.style.boxShadow = shadows.join(', ');
  pixelDiv.style.transform = 'scale(' + pixelSize + ')';
  pixelDiv.style.transformOrigin = 'top left';
  sprite.appendChild(pixelDiv);

  // Assemble
  pulseLayer.appendChild(sprite);
  spinWrapper.appendChild(pulseLayer);
  wrapper.appendChild(spinWrapper);

  wrapper.dataset.spawnX = xPos;
  wrapper.dataset.spawnY = yPos;
  wrapper.dataset.currentX = xPos;
  wrapper.dataset.currentY = yPos;
  wrapper.dataset.driftPath = JSON.stringify(drift);

  return wrapper;
}

// -------- Render / Update --------
let starCache = null;
let starContainer = null;

function initStarContainer() {
  if (starContainer) return;
  const container = document.createElement('div');
  container.id = 'starContainer';
  container.setAttribute('aria-hidden', 'true');
  document.body.insertBefore(container, document.body.firstChild);
  starContainer = container;
}

function renderBackgroundStars(profiles, settings, forceRebuild) {
  if (!starContainer) initStarContainer();

  const maxStars = Math.min(profiles.length, getMaxStars());

  if (!settings.enabled || maxStars === 0) {
    starContainer.innerHTML = '';
    starCache = null;
    return;
  }

  // If cache matches and not force, update CSS only
  if (!forceRebuild && starCache && starCache.length === maxStars) {
    updateStarCSS(settings);
    return;
  }

  // Full rebuild
  starContainer.innerHTML = '';
  const fragment = document.createDocumentFragment();
  const stars = [];

  for (let i = 0; i < maxStars; i++) {
    const star = createStar(profiles[i], settings, i);
    if (star) {
      fragment.appendChild(star);
      stars.push(star);
    }
  }

  starContainer.appendChild(fragment);
  starCache = stars;
}

function updateStarCSS(settings) {
  const stars = document.querySelectorAll('.star-drift-wrapper');
  const spinWrappers = document.querySelectorAll('.star-spin-wrapper');
  const pulseLayers = document.querySelectorAll('.star-pulse-layer');
  const sprites = document.querySelectorAll('.star-sprite');

  const size = settings.size;
  const opacity = settings.opacity / 100;
  const spinDuration = 60 / (settings.spinRPM / 10);

  // Update sprites (size & opacity)
  sprites.forEach(function(sprite) {
    const pixelDiv = sprite.querySelector('.pixel-grid');
    if (pixelDiv) {
      const cols = 12;
      const pixelSize = size / cols;
      pixelDiv.style.transform = 'scale(' + pixelSize + ')';
    }
    sprite.style.setProperty('--star-opacity', opacity);
  });

  // Update pulse
  pulseLayers.forEach(function(layer) {
    layer.style.setProperty('--pulse-duration', settings.pulseSpeed + 's');
  });

  // Update spin
  spinWrappers.forEach(function(wrapper) {
    wrapper.style.setProperty('--spin-duration', spinDuration + 's');
  });

  // Update drift (only if range/speed changed)
  const driftSpeed = settings.driftSpeed;
  const driftRange = settings.driftRange;
  stars.forEach(function(star, i) {
    const currentDuration = parseFloat(star.style.getPropertyValue('--drift-duration'));
    if (isNaN(currentDuration) || Math.abs(currentDuration - driftSpeed) > 2) {
      const drift = generateDriftPath(driftRange, driftSpeed);
      const kfName = injectDriftKeyframes(i, drift.waypoints, drift.duration);
      star.style.setProperty('--star-drift-name', kfName);
      star.style.setProperty('--drift-duration', drift.duration + 's');
    }
  });
}

// -------- Randomise / Recenter --------
function randomiseStars() {
  const container = document.getElementById('starContainer');
  if (!container) return;

  const stars = container.querySelectorAll('.star-drift-wrapper');
  if (stars.length === 0) return;

  requestAnimationFrame(function() {
    stars.forEach(function(star, i) {
      const x = 5 + Math.random() * 90;
      const y = 10 + Math.random() * 80;
      star.style.left = x + '%';
      star.style.top = y + '%';
      star.dataset.currentX = x;
      star.dataset.currentY = y;

      const drift = generateDriftPath(starSettings.driftRange, starSettings.driftSpeed);
      const kfName = injectDriftKeyframes(i, drift.waypoints, drift.duration);
      star.style.setProperty('--star-drift-name', kfName);
      star.style.setProperty('--drift-duration', drift.duration + 's');

      const spinWrapper = star.querySelector('.star-spin-wrapper');
      if (spinWrapper) {
        const direction = Math.random() > 0.5 ? 'normal' : 'reverse';
        spinWrapper.style.setProperty('--spin-direction', direction);
      }

      const sprite = star.querySelector('.star-sprite');
      if (sprite) {
        const opacity = (starSettings.opacity / 100) + (Math.random() - 0.5) * 0.04;
        sprite.style.setProperty('--star-opacity', Math.max(0.08, Math.min(0.45, opacity)));
      }
    });
  });

  addLog('Stars randomised!', 'info');
}

function recenterStars() {
  const container = document.getElementById('starContainer');
  if (!container) return;

  const stars = container.querySelectorAll('.star-drift-wrapper');
  if (stars.length === 0) return;

  requestAnimationFrame(function() {
    stars.forEach(function(star) {
      const homeX = star.dataset.spawnX || 50;
      const homeY = star.dataset.spawnY || 50;
      star.style.left = homeX + '%';
      star.style.top = homeY + '%';
      star.dataset.currentX = homeX;
      star.dataset.currentY = homeY;

      const sprite = star.querySelector('.star-sprite');
      if (sprite) {
        sprite.style.setProperty('--star-opacity', starSettings.opacity / 100);
      }
    });
  });

  addLog('Stars recentered', 'info');
}

// -------- Resize Handler --------
var resizeTimeout;

function handleResize() {
  clearTimeout(resizeTimeout);
  resizeTimeout = setTimeout(function() {
    if (!starSettings.enabled) return;
    const newMax = Math.min(profileList.length, getMaxStars());
    if (newMax !== (starCache ? starCache.length : 0)) {
      renderBackgroundStars(profileList, starSettings, true);
    }
  }, 300);
}

// -------- Bind Controls --------
function bindStarControls() {
  document.getElementById('starEnable').addEventListener('change', function() {
    starSettings.enabled = this.checked;
    saveStarSettings();
    renderBackgroundStars(profileList, starSettings);
  });

  const sizeSlider = document.getElementById('starSize');
  sizeSlider.addEventListener('input', function() {
    starSettings.size = parseInt(this.value);
    document.getElementById('starSizeVal').textContent = starSettings.size + 'px';
    saveStarSettings();
    updateStarCSS(starSettings);
  });

  document.querySelectorAll('.btn-star-size').forEach(function(btn) {
    btn.addEventListener('click', function() {
      const size = parseInt(this.dataset.size);
      sizeSlider.value = size;
      starSettings.size = size;
      document.getElementById('starSizeVal').textContent = size + 'px';
      saveStarSettings();
      updateStarCSS(starSettings);
    });
  });

  document.getElementById('starOpacity').addEventListener('input', function() {
    starSettings.opacity = parseInt(this.value);
    document.getElementById('starOpacityVal').textContent = starSettings.opacity + '%';
    saveStarSettings();
    updateStarCSS(starSettings);
  });

  var driftDebounce;
  document.getElementById('starDriftSpeed').addEventListener('input', function() {
    starSettings.driftSpeed = parseInt(this.value);
    document.getElementById('starDriftSpeedVal').textContent = starSettings.driftSpeed + 's';
    saveStarSettings();
    clearTimeout(driftDebounce);
    driftDebounce = setTimeout(function() {
      renderBackgroundStars(profileList, starSettings, true);
    }, 300);
  });

  document.getElementById('starSpinRPM').addEventListener('input', function() {
    starSettings.spinRPM = parseInt(this.value);
    var rpm = starSettings.spinRPM / 10;
    document.getElementById('starSpinRPMVal').textContent = rpm + ' RPM';
    saveStarSettings();
    updateStarCSS(starSettings);
  });

  document.getElementById('starPulseSpeed').addEventListener('input', function() {
    starSettings.pulseSpeed = parseInt(this.value) / 10;
    document.getElementById('starPulseSpeedVal').textContent = starSettings.pulseSpeed + 's';
    saveStarSettings();
    updateStarCSS(starSettings);
  });

  var rangeDebounce;
  document.getElementById('starDriftRange').addEventListener('input', function() {
    starSettings.driftRange = parseInt(this.value);
    document.getElementById('starDriftRangeVal').textContent = starSettings.driftRange + 'px';
    saveStarSettings();
    clearTimeout(rangeDebounce);
    rangeDebounce = setTimeout(function() {
      renderBackgroundStars(profileList, starSettings, true);
    }, 300);
  });

  document.getElementById('randomiseStarsBtn').addEventListener('click', randomiseStars);
  document.getElementById('resetStarsBtn').addEventListener('click', recenterStars);

  window.addEventListener('resize', handleResize);
}

// -------- Init Stars --------
function initBackgroundStars() {
  loadStarSettings();
  initStarContainer();
  bindStarControls();

  // Initial render after profiles load
  if (profileList && profileList.length > 0) {
    renderBackgroundStars(profileList, starSettings);
  }
}

// ============================================================
// MUSHROOM FAMILY FOOTER
// ============================================================

const mushroomContainer = document.getElementById('mushroomFooter');
const PIXEL_SIZE = 4;

const mushroomData = [
  {
    name: 'Toadstool',
    colors: {1:'#5c1a1a',2:'#e63946',3:'#ff7b89',4:'#ffffff',5:'#a98467',6:'#fefae0',7:'#ffffff',8:'#1d1d1d',9:'#ff99c8'},
    grid: [
      [0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0],
      [0,0,0,1,1,2,2,2,2,2,2,1,1,0,0,0],
      [0,0,1,2,2,4,4,2,2,4,4,2,2,1,0,0],
      [0,1,2,2,2,2,2,2,2,2,2,2,2,2,1,0],
      [1,2,2,4,4,2,2,2,2,2,2,4,4,2,2,1],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,8,6,6,8,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,9,6,6,9,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0]
    ]
  },
  {
    name: 'Chanterelle',
    colors: {1:'#8a5a19',2:'#f4a261',3:'#e9c46a',4:'#ffffff',5:'#8a5a19',6:'#fefae0',7:'#ffffff',8:'#1d1d1d',9:'#ff99c8'},
    grid: [
      [0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0],
      [0,0,0,1,2,2,3,3,3,3,2,2,1,0,0,0],
      [0,0,1,2,2,2,2,2,2,2,2,2,2,1,0,0],
      [0,1,2,4,2,2,2,4,2,2,2,4,2,2,1,0],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [1,2,2,2,4,2,2,2,2,2,4,2,2,2,2,1],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
      [0,0,0,0,5,5,5,5,5,5,5,5,0,0,0,0],
      [0,0,0,5,6,6,6,6,6,6,6,6,5,0,0,0],
      [0,0,0,5,6,8,6,6,6,6,8,6,5,0,0,0],
      [0,0,0,5,6,6,6,6,6,6,6,6,5,0,0,0],
      [0,0,0,5,6,6,9,6,6,9,6,6,5,0,0,0],
      [0,0,0,5,6,6,6,6,6,6,6,6,5,0,0,0],
      [0,0,0,0,5,5,5,5,5,5,5,5,0,0,0,0],
      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    ]
  },
  {
    name: 'Shiitake',
    colors: {1:'#3e2723',2:'#795548',3:'#a1887f',4:'#d7ccc8',5:'#3e2723',6:'#efebe9',7:'#ffffff',8:'#1d1d1d',9:'#ff99c8'},
    grid: [
      [0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0],
      [0,0,1,2,2,2,2,2,2,2,2,2,2,1,0,0],
      [0,1,2,2,3,2,2,2,2,2,2,3,2,2,1,0],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [1,2,4,2,2,2,2,2,2,2,2,2,2,4,2,1],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,8,6,6,8,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,9,6,6,9,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    ]
  },
  {
    name: 'Magic',
    colors: {1:'#2a1b38',2:'#7b2cbf',3:'#c77dff',4:'#e0aaff',5:'#2a1b38',6:'#f8edeb',7:'#ffffff',8:'#1d1d1d',9:'#ff99c8'},
    grid: [
      [0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0],
      [0,0,0,0,0,1,2,3,3,2,1,0,0,0,0,0],
      [0,0,0,0,1,2,2,2,2,2,2,1,0,0,0,0],
      [0,0,0,1,2,2,4,2,2,4,2,2,1,0,0,0],
      [0,0,1,2,2,2,2,2,2,2,2,2,2,1,0,0],
      [0,1,2,2,4,2,2,2,2,2,2,4,2,2,1,0],
      [1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1],
      [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,8,6,6,8,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,9,6,6,9,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    ]
  },
  {
    name: 'Morel',
    colors: {1:'#1b4332',2:'#40916c',3:'#74c69d',4:'#b7e4c7',5:'#1b4332',6:'#f8edeb',7:'#ffffff',8:'#1d1d1d',9:'#ff99c8'},
    grid: [
      [0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0],
      [0,0,0,1,2,3,2,3,2,3,2,2,1,0,0,0],
      [0,0,1,2,2,2,3,2,2,2,3,2,2,1,0,0],
      [0,1,2,3,2,2,2,3,2,2,2,3,2,2,1,0],
      [1,2,2,2,3,2,2,2,3,2,2,2,3,2,2,1],
      [1,2,3,2,2,3,2,2,2,3,2,2,2,3,2,1],
      [1,2,2,3,2,2,3,2,2,2,3,2,2,2,2,1],
      [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,8,6,6,8,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,5,6,9,6,6,9,6,5,0,0,0,0],
      [0,0,0,0,5,6,6,6,6,6,6,5,0,0,0,0],
      [0,0,0,0,0,5,5,5,5,5,5,0,0,0,0,0],
      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    ]
  }
];

function renderMushroom(data) {
  const wrapper = document.createElement('div');
  wrapper.className = 'mushroom-wrapper';
  const sprite = document.createElement('div');
  sprite.className = 'mushroom-sprite';
  for (let r = 0; r < 16; r++) {
    for (let c = 0; c < 16; c++) {
      const val = data.grid[r][c];
      if (val === 0) continue;
      const pixel = document.createElement('div');
      pixel.className = 'pixel';
      pixel.style.backgroundColor = data.colors[val];
      pixel.style.top = (r * PIXEL_SIZE) + 'px';
      pixel.style.left = (c * PIXEL_SIZE) + 'px';
      sprite.appendChild(pixel);
    }
  }
  wrapper.appendChild(sprite);
  const shadow = document.createElement('div');
  shadow.className = 'mushroom-shadow';
  wrapper.appendChild(shadow);
  return wrapper;
}

mushroomData.forEach(function(data) {
  mushroomContainer.appendChild(renderMushroom(data));
});

connectWS();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// Web Server Handlers
// ============================================================

static void handleRoot() {
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send_P(200, "text/html", INDEX_HTML);
}

static void handleNotFound() {
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(404, "text/plain", "404 Not Found");
}

// ============================================================
// WebSocket Handler
// ============================================================

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.print(F("[WS] Client disconnected: "));
      Serial.println(num);
      break;

    case WStype_CONNECTED:
      Serial.print(F("[WS] Client connected: "));
      Serial.println(num);
      sendSensorUpdate();
      sendSystemStatus();
      sendConfigUpdate(num);
      sendCalibrationUpdate();
      break;

    case WStype_TEXT: {
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, payload, length);

      if (error) {
        Serial.print(F("[WS] JSON parse error: "));
        Serial.println(error.c_str());
        return;
      }

      uint8_t msgType = doc["type"] | 0;
      const char* cmd = doc["cmd"] | "";

      if (msgType == WS_GRAPH_DATA) {
        static unsigned long g_lastGraphRequest[MAX_WS_CLIENTS] = {0};
        unsigned long now = millis();
        if (num >= MAX_WS_CLIENTS) return;
        if (now - g_lastGraphRequest[num] < GRAPH_RATE_LIMIT_MS) return;
        g_lastGraphRequest[num] = now;

        GraphDataRequest req;
        req.sensorType = doc["sensor"] | 0;
        req.startEpoch = doc["start"] | 0;
        req.endEpoch = doc["end"] | 0;
        req.maxPoints = doc["max"] | GRAPH_MAX_RESPONSE_POINTS;
        req.requestId = doc["rid"] | 0;

        if (req.sensorType > 3) return;
        if (req.startEpoch < 1577836800) req.startEpoch = 1577836800;
        if (req.endEpoch > 0 && req.startEpoch >= req.endEpoch) return;
        if (req.maxPoints == 0 || req.maxPoints > GRAPH_MAX_RESPONSE_POINTS) req.maxPoints = GRAPH_MAX_RESPONSE_POINTS;

        char* graphOutput = (char*)heap_caps_malloc(GRAPH_RESPONSE_BUFFER_SIZE, MALLOC_CAP_8BIT);
        if (!graphOutput) {
          char errOutput[64];
          snprintf(errOutput, sizeof(errOutput),
                   "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[]}",
                   req.sensorType, req.requestId);
          g_webSocket.sendTXT(num, (const uint8_t*)errOutput, strlen(errOutput));
          break;
        }

        size_t len = sdLogger_getHistoricalData(&req, graphOutput, GRAPH_RESPONSE_BUFFER_SIZE);
        if (len > 0) {
          g_webSocket.sendTXT(num, (const uint8_t*)graphOutput, len);
        } else {
          char errOutput[64];
          snprintf(errOutput, sizeof(errOutput),
                   "{\"type\":100,\"s\":%d,\"rid\":%u,\"p\":[]}",
                   req.sensorType, req.requestId);
          g_webSocket.sendTXT(num, (const uint8_t*)errOutput, strlen(errOutput));
        }
        heap_caps_free(graphOutput);
        break;
      }

      if (msgType == WS_COMMAND && strcmp(cmd, "relay") == 0) {
        uint8_t index = doc["index"] | 0;
        bool state = (doc["state"].as<int>() != 0);
        bool force = (doc["force"].as<int>() != 0);
        bool confirmed = (doc["confirmed"].as<int>() != 0);
        bool identify = (doc["identify"].as<int>() != 0);

        if (index >= RELAY_COUNT) {
          Serial.print(F("[WS] Invalid relay index: "));
          Serial.println(index);
          return;
        }

        if (identify) {
          relayManager_identifyRelay(index);
          return;
        }

        if (state && !confirmed && relayManager_isRelayLoud(index)) {
          bool nightMode;
          portENTER_CRITICAL(&g_stateMux);
          nightMode = g_systemState.nightModeActive;
          portEXIT_CRITICAL(&g_stateMux);

          if (nightMode) {
            StaticJsonDocument<128> confirmDoc;
            confirmDoc["type"] = 2;
            confirmDoc["message"] = "CONFIRM_LOUD_NIGHT";
            confirmDoc["level"] = "warn";
            confirmDoc["relay"] = index;
            char confirmOutput[128];
            size_t confirmLen = serializeJson(confirmDoc, confirmOutput, sizeof(confirmOutput));
            g_webSocket.sendTXT(num, (const uint8_t*)confirmOutput, confirmLen);
            return;
          }
        }

        if (state && confirmed && index == RELAY_COMPRESSOR) {
          bool nightMode;
          portENTER_CRITICAL(&g_stateMux);
          nightMode = g_systemState.nightModeActive;
          portEXIT_CRITICAL(&g_stateMux);
          if (nightMode) {
            automation_activateCompressorOverride();
          }
        }

        bool calibrationActive;
        portENTER_CRITICAL(&g_stateMux);
        calibrationActive = g_systemState.calibrationActive;
        portEXIT_CRITICAL(&g_stateMux);

        if (!calibrationActive) {
          if (index == RELAY_HOH || index == RELAY_AIR_ASSIST) {
            if (!automation_isHumidityOverrideActive()) {
              automation_activateHumidityOverride();
            }
          } else if (index == RELAY_EXHAUST) {
            if (!automation_isCO2OverrideActive()) {
              automation_activateCO2Override();
            }
          }

          if (relayManager_setRelay(index, state, force)) {
            Serial.print(F("[WS] Relay "));
            Serial.print(index);
            Serial.print(F(" -> "));
            Serial.print(state ? "ON" : "OFF");
            if (force) Serial.print(F(" (forced)"));
            Serial.println();
          }
        }
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "thresholds_and_ema") == 0) {
        AutomationThresholds* thresholds = automation_getThresholds();
        AutomationThresholds newThresholds = *thresholds;

        newThresholds.humHoHFloor = constrain(doc["data"]["humHoHFloor"] | thresholds->humHoHFloor, 0, 100);
        newThresholds.humAssistFloor = constrain(doc["data"]["humAssistFloor"] | thresholds->humAssistFloor, 0, 100);
        newThresholds.humCeiling = constrain(doc["data"]["humCeiling"] | thresholds->humCeiling, 0, 100);
        newThresholds.humExhaustOn = constrain(doc["data"]["humExhaustOn"] | thresholds->humExhaustOn, 0, 100);
        newThresholds.assistOnSec = max(doc["data"]["assistOnSec"] | thresholds->assistOnSec, 0);
        newThresholds.assistOffSec = max(doc["data"]["assistOffSec"] | thresholds->assistOffSec, 0);
        newThresholds.co2HighLimit = max(doc["data"]["co2HighLimit"] | thresholds->co2HighLimit, 0);
        newThresholds.co2LowTarget = max(doc["data"]["co2LowTarget"] | thresholds->co2LowTarget, 0);
        newThresholds.co2Emergency = max(doc["data"]["co2Emergency"] | thresholds->co2Emergency, 0);

        automation_updateThresholds(&newThresholds);

        float weight = doc["data"]["emaWeight"] | DEFAULT_EMA_WEIGHT;
        if (weight < EMA_WEIGHT_MIN) weight = EMA_WEIGHT_MIN;
        if (weight > EMA_WEIGHT_MAX) weight = EMA_WEIGHT_MAX;
        adaptive_setEMAWeight(weight);
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "rtc") == 0) {
        const char* datetime = doc["datetime"] | "";
        if (strlen(datetime) == 0) {
          Serial.println(F("[WS] RTC command received with empty datetime"));
          return;
        }
        int year, month, day, hour, minute;
        if (sscanf(datetime, "%d-%d-%dT%d:%d", &year, &month, &day, &hour, &minute) != 5) {
          Serial.print(F("[WS] Failed to parse datetime: "));
          Serial.println(datetime);
          return;
        }
        if (year < 2000 || year > 2099 || month < 1 || month > 12 ||
            day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59) {
          Serial.print(F("[WS] RTC datetime out of range: "));
          Serial.println(datetime);
          return;
        }
        int daysInMonth[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
        if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
          daysInMonth[2] = 29;
        }
        if (day > daysInMonth[month]) {
          Serial.print(F("[WS] Invalid day for month: "));
          Serial.println(datetime);
          return;
        }
        if (rtc_setTime((uint8_t)hour, (uint8_t)minute, 0,
                        (uint8_t)day, (uint8_t)month, (uint16_t)year, 1)) {
          Serial.println(F("[WS] RTC time set successfully from Web UI"));
          StaticJsonDocument<128> responseDoc;
          responseDoc["type"] = 2;
          responseDoc["message"] = "RTC time updated successfully";
          responseDoc["level"] = "info";
          char response[128];
          size_t responseLen = serializeJson(responseDoc, response, sizeof(response));
          g_webSocket.sendTXT(num, (const uint8_t*)response, responseLen);
        } else {
          Serial.println(F("[WS] RTC time set FAILED"));
          StaticJsonDocument<128> responseDoc;
          responseDoc["type"] = 2;
          responseDoc["message"] = "Failed to set RTC time - check date values";
          responseDoc["level"] = "warn";
          char response[128];
          size_t responseLen = serializeJson(responseDoc, response, sizeof(response));
          g_webSocket.sendTXT(num, (const uint8_t*)response, responseLen);
        }
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "calibrate_start") == 0) {
        adaptive_startCalibration();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "calibrate_cancel") == 0) {
        adaptive_cancelCalibration();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "resume_automation") == 0) {
        automation_deactivateAllOverrides();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "warmup") == 0) {
        unsigned long duration = doc["duration"] | 0;
        if (duration > 600) duration = 600;

        bool alreadySelected = false;

        portENTER_CRITICAL(&g_stateMux);
        alreadySelected = g_warmupSelected;
        portEXIT_CRITICAL(&g_stateMux);

        if (alreadySelected) {
          Serial.println(F("[WS] Warmup already selected, ignoring"));
          return;
        }

        relayManager_setRelay(RELAY_COMPRESSOR, true);

        unsigned long start = (duration > 0) ? millis() : 0;
        unsigned long durMs = duration * 1000UL;

        portENTER_CRITICAL(&g_stateMux);
        if (!g_warmupSelected) {
          g_compressorWarmupStart = start;
          g_compressorWarmupDuration = durMs;
          g_warmupSelected = true;
        }
        portEXIT_CRITICAL(&g_stateMux);

        if (duration > 0) {
          Serial.print(F("[BOOT] Compressor warmup: "));
          Serial.print(duration);
          Serial.println(F(" seconds"));
        } else {
          Serial.println(F("[BOOT] Compressor warmup skipped"));
        }
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "relay_mapping") == 0) {
        const RelayMapping* current = relayManager_getMapping();
        RelayMapping newMapping;
        newMapping.magic = RELAY_MAPPING_MAGIC;
        newMapping.pinPos1 = current->pinPos1;
        newMapping.pinPos2 = current->pinPos2;
        newMapping.pinPos3 = current->pinPos3;
        newMapping.pinPos4 = current->pinPos4;
        newMapping.functionForPos[0] = doc["data"]["funcPos1"] | current->functionForPos[0];
        newMapping.functionForPos[1] = doc["data"]["funcPos2"] | current->functionForPos[1];
        newMapping.functionForPos[2] = doc["data"]["funcPos3"] | current->functionForPos[2];
        newMapping.functionForPos[3] = doc["data"]["funcPos4"] | current->functionForPos[3];

        if (relayManager_updateMapping(&newMapping)) {
          memcpy(&g_runtimeCache.relayMapping, relayManager_getMapping(), sizeof(RelayMapping));
          sdLogger_saveCache();

          StaticJsonDocument<128> responseDoc;
          responseDoc["type"] = 2;
          responseDoc["message"] = "Relay mapping applied and saved";
          responseDoc["level"] = "info";
          char response[128];
          size_t responseLen = serializeJson(responseDoc, response, sizeof(response));
          g_webSocket.sendTXT(num, (const uint8_t*)response, responseLen);
               } else {
          StaticJsonDocument<128> responseDoc;
          responseDoc["type"] = 2;
          responseDoc["message"] = "Invalid relay mapping — check functions";
          responseDoc["level"] = "warn";
          char response[128];
          size_t responseLen = serializeJson(responseDoc, response, sizeof(response));
          g_webSocket.sendTXT(num, (const uint8_t*)response, responseLen);
        }
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "simulate") == 0) {
        float current = doc["current"] | 0.0f;
        float target = doc["target"] | 88.0f;
        float delta = target - current;

        StaticJsonDocument<128> responseDoc;
        responseDoc["type"] = 99;

        if (delta > 0) {
          float recoveryTime = adaptive_projectRecoveryTime(delta);
          char simResult[64];
          snprintf(simResult, sizeof(simResult), "%.0f seconds (%.1f minutes)", recoveryTime, recoveryTime / 60.0f);
          responseDoc["simResult"] = simResult;
        } else {
          responseDoc["simResult"] = "Already at or above target";
        }

        char response[128];
        size_t responseLen = serializeJson(responseDoc, response, sizeof(response));
        if (responseLen >= sizeof(response)) {
          Serial.println(F("[WS] WARNING: Simulation response JSON truncated"));
        }
        g_webSocket.sendTXT(num, (const uint8_t*)response, responseLen);
      }

      // ============================================================
      // Mascots Profile Commands
      // ============================================================

      else if (msgType == WS_COMMAND && strcmp(cmd, "profile_list") == 0) {
        handleProfileList(num);
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "profile_save") == 0) {
        handleProfileSave(num, doc);
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "profile_load") == 0) {
        handleProfileLoad(num, doc);
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "profile_delete") == 0) {
        handleProfileDelete(num, doc);
      }

      break;
    }

    default:
      break;
  }
}

// ============================================================
// WebSocket Push Helpers
// ============================================================

static void sendSensorUpdate() {
  float temp, hum, fridgeTemp;
  uint16_t co2;
  bool tempFault, humFault, co2Fault, nightMode, calibrationActive;
  uint8_t activeBand;
  float confidence;

  portENTER_CRITICAL(&g_stateMux);
  temp = g_systemState.currentTemp;
  hum = g_systemState.currentHumidity;
  co2 = g_systemState.currentCO2;
  tempFault = g_systemState.tempSensorFault;
  humFault = g_systemState.humiditySensorFault;
  co2Fault = g_systemState.co2SensorFault;
  nightMode = g_systemState.nightModeActive;
  calibrationActive = g_systemState.calibrationActive;
  portEXIT_CRITICAL(&g_stateMux);

  bool warmupSelected;
  unsigned long warmupDuration;
  portENTER_CRITICAL(&g_stateMux);
  warmupSelected = g_warmupSelected;
  warmupDuration = g_compressorWarmupDuration;
  portEXIT_CRITICAL(&g_stateMux);

  fridgeTemp = network_getFridgeTemp();
  bool fridgeLost = network_isFridgeHeartbeatLost();
  bool wifiConnected = network_isWiFiConnected();
  bool apMode = network_isAPMode();
  activeBand = adaptive_getCurrentBand();

  BandProfile* profile = adaptive_getActiveProfile();
  confidence = profile ? profile->confidenceScore : 0.0f;

  char timeStr[24];
  rtc_getTimeString(timeStr, sizeof(timeStr));

  const char* controlMode = "Bang-Bang";
  if (calibrationActive) {
    controlMode = "Calibration";
  } else if (profile && profile->valid && profile->confidenceScore >= PID_AUTO_ENABLE_CONFIDENCE) {
    controlMode = "PID";
  }

  StaticJsonDocument<1024> doc;
  doc["type"] = WS_SENSOR_UPDATE;
  doc["temp"] = temp;
  doc["hum"] = hum;
  doc["co2"] = co2;
  doc["fridge"] = fridgeTemp;
  doc["fridgeHum"] = network_getFridgeHumidity();
  doc["fridgeDoor"] = network_isFridgeDoorOpen();
  doc["tempFault"] = tempFault;
  doc["humFault"] = humFault;
  doc["co2Fault"] = co2Fault;
  doc["nightMode"] = nightMode;
  doc["wifiConnected"] = wifiConnected;
  doc["apMode"] = apMode;
  doc["rtcTime"] = timeStr;
  doc["fridgeLost"] = fridgeLost;
  doc["activeBand"] = activeBand;
  doc["confidence"] = confidence;
  doc["controlMode"] = controlMode;
  doc["weatherValid"] = g_systemState.weatherValid;
  doc["weatherTemp"] = g_systemState.weatherTemp;
  doc["weatherHum"] = g_systemState.weatherHum;
  doc["weatherCode"] = g_systemState.weatherCode;
  doc["weatherWind"] = g_systemState.weatherWind;
  doc["weatherStale"] = (g_systemState.weatherValid && 
                         (millis() - g_systemState.weatherLastFetch > WEATHER_STALE_MS));

  doc["warmupSelected"] = warmupSelected;
  if (warmupSelected && warmupDuration > 0) {
    doc["warmupDuration"] = warmupDuration / 1000;
  }

  char output[768];
  size_t len = serializeJson(doc, output, sizeof(output));
  if (len >= sizeof(output)) {
    Serial.println(F("[WS] WARNING: Sensor update JSON truncated — increase buffer size"));
    return;
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, len);
}

static void sendSystemStatus() {
  bool hoh, assist, fan, compressor;

  portENTER_CRITICAL(&g_stateMux);
  hoh = g_systemState.hoHActive;
  assist = g_systemState.airAssistActive;
  fan = g_systemState.exhaustFanActive;
  compressor = g_systemState.compressorActive;
  portEXIT_CRITICAL(&g_stateMux);

  bool warmupSelected;
  unsigned long warmupStart, warmupDuration;
  unsigned long now = millis();
  portENTER_CRITICAL(&g_stateMux);
  warmupSelected = g_warmupSelected;
  warmupStart = g_compressorWarmupStart;
  warmupDuration = g_compressorWarmupDuration;
  portEXIT_CRITICAL(&g_stateMux);

  StaticJsonDocument<256> doc;
  doc["type"] = WS_RELAY_STATE;
  doc["hoh"] = hoh;
  doc["assist"] = assist;
  doc["fan"] = fan;
  doc["compressor"] = compressor;
  doc["humOverride"] = automation_isHumidityOverrideActive();
  doc["humOverrideRemaining"] = automation_getHumidityOverrideRemaining() / 1000;
  doc["co2Override"] = automation_isCO2OverrideActive();
  doc["co2OverrideRemaining"] = automation_getCO2OverrideRemaining() / 1000;
  doc["warmupSelected"] = warmupSelected;
  if (warmupSelected && warmupDuration > 0) {
    unsigned long elapsed = now - warmupStart;
    doc["warmupRemaining"] = (elapsed < warmupDuration) ?
        (warmupDuration - elapsed) / 1000 : 0;
  }
  doc["compressorLocked"] = relayManager_isCompressorLocked();
  doc["alerts"] = safety_getActiveAlerts() | network_getActiveAlerts();

  char output[256];
  size_t len = serializeJson(doc, output, sizeof(output));
  if (len >= sizeof(output)) {
    Serial.println(F("[WS] WARNING: System status JSON truncated — increase buffer size"));
    return;
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, len);
}

static void sendConfigUpdate(uint8_t clientNum) {
  AutomationThresholds* t = automation_getThresholds();

  StaticJsonDocument<256> doc;
  doc["type"] = WS_THRESHOLD_UPDATE;
  doc["humHoHFloor"] = t->humHoHFloor;
  doc["humAssistFloor"] = t->humAssistFloor;
  doc["humCeiling"] = t->humCeiling;
  doc["humExhaustOn"] = t->humExhaustOn;
  doc["assistOnSec"] = t->assistOnSec;
  doc["assistOffSec"] = t->assistOffSec;
  doc["co2HighLimit"] = t->co2HighLimit;
  doc["co2LowTarget"] = t->co2LowTarget;
  doc["co2Emergency"] = t->co2Emergency;
  doc["emaWeight"] = adaptive_getEMAWeight();
  const RelayMapping* mapping = relayManager_getMapping();
  doc["funcPos1"] = mapping->functionForPos[0];
  doc["funcPos2"] = mapping->functionForPos[1];
  doc["funcPos3"] = mapping->functionForPos[2];
  doc["funcPos4"] = mapping->functionForPos[3];
  doc["weatherLat"] = g_runtimeCache.weatherLat;
  doc["weatherLon"] = g_runtimeCache.weatherLon;
  doc["overrideTimeout"] = g_runtimeCache.overrideTimeoutMinutes;
  char output[256];
  size_t len = serializeJson(doc, output, sizeof(output));
  if (len >= sizeof(output)) {
    Serial.println(F("[WS] WARNING: Config update JSON truncated — increase buffer size"));
    return;
  }
  g_webSocket.sendTXT(clientNum, (const uint8_t*)output, len);
}

static void sendCalibrationUpdate() {
  bool active = adaptive_isCalibrating();

  StaticJsonDocument<128> calibDoc;
  calibDoc["type"] = WS_CALIBRATION_STATUS;
  calibDoc["active"] = active;

  if (active) {
    unsigned long elapsed = millis() - adaptive_getCalibrationStartTime();
    unsigned long remaining;
    if (elapsed >= (CALIBRATION_TOTAL_SEC * 1000UL)) {
      remaining = 0;
    } else {
      remaining = (CALIBRATION_TOTAL_SEC * 1000UL) - elapsed;
    }
    calibDoc["remaining"] = remaining / 1000;
    calibDoc["band"] = adaptive_getCurrentBand();
  } else {
    calibDoc["remaining"] = 0;
    calibDoc["band"] = 0;
  }

  char output[128];
  size_t len = serializeJson(calibDoc, output, sizeof(output));
  if (len >= sizeof(output)) {
    Serial.println(F("[WS] WARNING: Calibration update JSON truncated — increase buffer size"));
    return;
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, len);
}

// ============================================================
// Mascots Tab — Profile Helper Functions
// ============================================================

static LoadResult loadProfilesJson(JsonDocument& doc, bool& wasRecovered) {
  wasRecovered = false;

  if (!SPIFFS.exists(PROFILES_FILE)) {
    doc.to<JsonObject>();
    return LoadResult::Empty;
  }

  File file = SPIFFS.open(PROFILES_FILE, "r");
  if (!file) {
    Serial.println(F("[PROFILES] Failed to open file for reading"));
    return LoadResult::Failed;
  }

  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.println(F("[PROFILES] JSON parse error — recovering..."));
    Serial.print(F("[PROFILES] Error: "));
    Serial.println(error.c_str());

    wasRecovered = true;

    if (SPIFFS.exists(PROFILES_BACKUP)) SPIFFS.remove(PROFILES_BACKUP);
    SPIFFS.rename(PROFILES_FILE, PROFILES_BACKUP);

    File fresh = SPIFFS.open(PROFILES_FILE, "w");
    if (fresh) { fresh.print("{}"); fresh.close(); Serial.println(F("[PROFILES] Created fresh empty file")); }

    doc.to<JsonObject>();
    return LoadResult::Recovered;
  }

  return LoadResult::OK;
}

static bool saveProfilesJson(const JsonDocument& doc) {
  File file = SPIFFS.open(PROFILES_TEMP, "w");
  if (!file) {
    Serial.println(F("[PROFILES] Failed to open temp file for writing"));
    return false;
  }

  size_t jsonSize = measureJson(doc);
  size_t bytesWritten = serializeJson(doc, file);
  file.close();

  if (bytesWritten == 0 || bytesWritten != jsonSize) {
    Serial.print(F("[PROFILES] Write mismatch: expected "));
    Serial.print(jsonSize);
    Serial.print(F(", got "));
    Serial.println(bytesWritten);
    SPIFFS.remove(PROFILES_TEMP);
    return false;
  }

  DynamicJsonDocument verifyDoc(8192);
  File verifyFile = SPIFFS.open(PROFILES_TEMP, "r");
  if (!verifyFile) {
    Serial.println(F("[PROFILES] Failed to open temp file for verification"));
    SPIFFS.remove(PROFILES_TEMP);
    return false;
  }

  DeserializationError error = deserializeJson(verifyDoc, verifyFile);
  verifyFile.close();

  if (error) {
    Serial.println(F("[PROFILES] Temp file verification failed — corrupt write"));
    SPIFFS.remove(PROFILES_TEMP);
    return false;
  }

  if (!SPIFFS.rename(PROFILES_TEMP, PROFILES_FILE)) {
    Serial.println(F("[PROFILES] Atomic rename failed"));
    SPIFFS.remove(PROFILES_TEMP);
    return false;
  }

  return true;
}

static void getProfileNames(JsonArray& names) {
  bool wasRecovered;
  DynamicJsonDocument doc(8192);
  LoadResult result = loadProfilesJson(doc, wasRecovered);
  if (result == LoadResult::Failed) return;

  JsonObject obj = doc.as<JsonObject>();
  for (JsonPair kv : obj) {
    names.add(kv.key().c_str());
  }
}

static bool isValidTemplate(const char* name) {
  static const char* validTemplates[] = {"amanita", "chanterelle", "shiitake", "magic", "morel"};
  for (size_t i = 0; i < 5; i++) {
    if (strcmp(name, validTemplates[i]) == 0) return true;
  }
  return false;
}

static bool validateProfileData(const JsonObject& data) {
  if (!data.containsKey("template")) return false;
  if (!data.containsKey("capWidth")) return false;
  if (!data.containsKey("capHeight")) return false;
  if (!data.containsKey("capColor")) return false;
  if (!data.containsKey("stemHeight")) return false;
  if (!data.containsKey("stemWidth")) return false;
  if (!data.containsKey("stemColor")) return false;
  if (!data.containsKey("spots")) return false;
  if (!data.containsKey("bounceHeight")) return false;
  if (!data.containsKey("animSpeed")) return false;

  const char* templateName = data["template"] | "";
  if (!isValidTemplate(templateName)) return false;

  int capWidth = data["capWidth"] | 0;
  if (capWidth < 60 || capWidth > 120) return false;
  int capHeight = data["capHeight"] | 0;
  if (capHeight < 60 || capHeight > 120) return false;
  int stemHeight = data["stemHeight"] | 0;
  if (stemHeight < 60 || stemHeight > 140) return false;
  int stemWidth = data["stemWidth"] | 0;
  if (stemWidth < 50 || stemWidth > 120) return false;
  int spots = data["spots"] | -1;
  if (spots != 0 && spots != 2 && spots != 4 && spots != 6) return false;
  int bounceHeight = data["bounceHeight"] | 0;
  if (bounceHeight < 5 || bounceHeight > 35) return false;
  int animSpeed = data["animSpeed"] | 0;
  if (animSpeed < 12 || animSpeed > 35) return false;

  const char* capColor = data["capColor"] | "";
  if (strlen(capColor) != 7 || capColor[0] != '#') return false;
  for (int i = 1; i < 7; i++) {
    if (!isxdigit((unsigned char)capColor[i])) return false;
  }

  const char* stemColor = data["stemColor"] | "";
  if (strlen(stemColor) != 7 || stemColor[0] != '#') return false;
  for (int i = 1; i < 7; i++) {
    if (!isxdigit((unsigned char)stemColor[i])) return false;
  }

  return true;
}

static void sendProfileResponse(uint8_t num, const char* cmd, const char* status, const char* message) {
  DynamicJsonDocument resp(256);
  resp["type"] = WS_PROFILE_RESPONSE;
  resp["cmd"] = cmd;
  resp["status"] = status;
  resp["message"] = message;
  String output;
  serializeJson(resp, output);
  g_webSocket.sendTXT(num, (const uint8_t*)output.c_str(), output.length());
}

// ============================================================
// Mascots Tab — WebSocket Command Handlers
// ============================================================

static void handleProfileList(uint8_t num) {
  DynamicJsonDocument response(1024);
  response["type"] = WS_PROFILE_RESPONSE;
  response["cmd"] = "profile_list";

  JsonArray names = response.createNestedArray("names");
  getProfileNames(names);

  String output;
  serializeJson(response, output);
  g_webSocket.sendTXT(num, (const uint8_t*)output.c_str(), output.length());
}

static void handleProfileSave(uint8_t num, const JsonDocument& req) {
  const char* name = req["name"] | "";
  size_t nameLen = strlen(name);

  if (nameLen == 0 || nameLen > 32) {
    sendProfileResponse(num, "profile_save", "error", "Name must be 1-32 chars");
    return;
  }

  for (size_t i = 0; i < nameLen; i++) {
    char c = name[i];
    if (!isalnum((unsigned char)c) && c != ' ' && c != '-' && c != '_' && c != '.') {
      sendProfileResponse(num, "profile_save", "error", "Invalid name (use letters, numbers, spaces, - _ .)");
      return;
    }
  }

  JsonObject data = req["data"].as<JsonObject>();
  if (data.isNull()) {
    sendProfileResponse(num, "profile_save", "error", "Missing profile data");
    return;
  }

  if (!validateProfileData(data)) {
    sendProfileResponse(num, "profile_save", "error", "Invalid profile data");
    return;
  }

  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;
  if (freeBytes < 2048) {
    sendProfileResponse(num, "profile_save", "error", "Storage full (need 2KB free)");
    return;
  }

  if (g_profileMutex == NULL) {
    sendProfileResponse(num, "profile_save", "error", "Storage not initialized");
    return;
  }

  if (xSemaphoreTake(g_profileMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    sendProfileResponse(num, "profile_save", "error", "Storage busy, try again");
    return;
  }

  bool wasRecovered;
  DynamicJsonDocument doc(8192);
  LoadResult result = loadProfilesJson(doc, wasRecovered);

  if (result == LoadResult::Failed) {
    xSemaphoreGive(g_profileMutex);
    sendProfileResponse(num, "profile_save", "error", "Storage read error");
    return;
  }

  JsonObject obj = doc.as<JsonObject>();

  if (!obj.containsKey(name) && obj.size() >= MAX_PROFILES) {
    xSemaphoreGive(g_profileMutex);
    sendProfileResponse(num, "profile_save", "error", "Profile limit reached (8 max)");
    return;
  }

  obj[name] = data;
  bool success = saveProfilesJson(doc);

  xSemaphoreGive(g_profileMutex);

  if (success) {
    sendProfileResponse(num, "profile_save", "ok", wasRecovered ? "Profile saved (recovered from corruption)" : "Profile saved");
  } else {
    sendProfileResponse(num, "profile_save", "error", "Disk write failed");
  }
}

static void handleProfileLoad(uint8_t num, const JsonDocument& req) {
  const char* name = req["name"] | "";
  if (strlen(name) == 0) {
    sendProfileResponse(num, "profile_load", "error", "Name is required");
    return;
  }

  if (g_profileMutex == NULL) {
    sendProfileResponse(num, "profile_load", "error", "Storage not initialized");
    return;
  }

  if (xSemaphoreTake(g_profileMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    sendProfileResponse(num, "profile_load", "error", "Storage busy, try again");
    return;
  }

  bool wasRecovered;
  DynamicJsonDocument doc(8192);
  LoadResult result = loadProfilesJson(doc, wasRecovered);

  if (result == LoadResult::Failed) {
    xSemaphoreGive(g_profileMutex);
    sendProfileResponse(num, "profile_load", "error", "Storage read error");
    return;
  }

  JsonObject obj = doc.as<JsonObject>();
  if (!obj.containsKey(name)) {
    xSemaphoreGive(g_profileMutex);
    sendProfileResponse(num, "profile_load", "error", "Profile not found");
    return;
  }

  DynamicJsonDocument response(4096);
  response["type"] = WS_PROFILE_RESPONSE;
  response["cmd"] = "profile_load";
  response["name"] = name;
  response["data"].set(obj[name]);

  xSemaphoreGive(g_profileMutex);

  String output;
  serializeJson(response, output);
  g_webSocket.sendTXT(num, (const uint8_t*)output.c_str(), output.length());

  if (wasRecovered) {
    sendProfileResponse(num, "profile_load", "ok", "Profile loaded (recovered from corruption)");
  }
}

static void handleProfileDelete(uint8_t num, const JsonDocument& req) {
  const char* name = req["name"] | "";
  if (strlen(name) == 0) {
    sendProfileResponse(num, "profile_delete", "error", "Name is required");
    return;
  }

  if (g_profileMutex == NULL) {
    sendProfileResponse(num, "profile_delete", "error", "Storage not initialized");
    return;
  }

  if (xSemaphoreTake(g_profileMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    sendProfileResponse(num, "profile_delete", "error", "Storage busy, try again");
    return;
  }

  bool wasRecovered;
  DynamicJsonDocument doc(8192);
  LoadResult result = loadProfilesJson(doc, wasRecovered);

  if (result == LoadResult::Failed) {
    xSemaphoreGive(g_profileMutex);
    sendProfileResponse(num, "profile_delete", "error", "Storage read error");
    return;
  }

  JsonObject obj = doc.as<JsonObject>();
  if (!obj.containsKey(name)) {
    xSemaphoreGive(g_profileMutex);
    sendProfileResponse(num, "profile_delete", "error", "Profile not found");
    return;
  }

  obj.remove(name);
  bool success = saveProfilesJson(doc);

  xSemaphoreGive(g_profileMutex);

  if (success) {
    sendProfileResponse(num, "profile_delete", "ok", "Profile deleted");
  } else {
    sendProfileResponse(num, "profile_delete", "error", "Disk write failed");
  }
}

// ============================================================
// Startup Recovery
// ============================================================

static void recoverProfiles() {
  if (SPIFFS.exists(PROFILES_TEMP) && !SPIFFS.exists(PROFILES_FILE)) {
    SPIFFS.rename(PROFILES_TEMP, PROFILES_FILE);
    Serial.println(F("[PROFILES] Recovered from temp file"));
  }

  if (SPIFFS.exists(PROFILES_BACKUP) && !SPIFFS.exists(PROFILES_FILE)) {
    SPIFFS.rename(PROFILES_BACKUP, PROFILES_FILE);
    Serial.println(F("[PROFILES] Recovered from backup file"));
  }

  if (SPIFFS.exists(PROFILES_TEMP)) {
    SPIFFS.remove(PROFILES_TEMP);
  }
}

// ============================================================
// Public API
// ============================================================

bool webUI_init() {
  Serial.println(F("[WEB] Initializing web server..."));

  if (!SPIFFS.begin(true)) {
    Serial.println(F("[WEB] SPIFFS mount failed"));
    return false;
  }

  g_profileMutex = xSemaphoreCreateMutex();
  if (g_profileMutex == NULL) {
    Serial.println(F("[PROFILES] Failed to create mutex"));
  }

  recoverProfiles();

  if (!SPIFFS.exists(PROFILES_FILE)) {
    File file = SPIFFS.open(PROFILES_FILE, "w");
    if (file) { file.print("{}"); file.close(); Serial.println(F("[PROFILES] Created empty profiles file")); }
    else { Serial.println(F("[PROFILES] Failed to create profiles file")); }
  }

  g_server.on("/chart-4.4.0.min.js", []() {
    if (SPIFFS.exists("/chart-4.4.0.min.js")) {
      g_server.sendHeader("Cache-Control", "max-age=31536000, immutable");
      File f = SPIFFS.open("/chart-4.4.0.min.js", "r");
      g_server.streamFile(f, "application/javascript");
      f.close();
    } else {
      g_server.send(404, "text/plain", "Not Found");
    }
  });
  g_server.on("/", handleRoot);
  g_server.onNotFound(handleNotFound);

  g_server.begin();
  Serial.print(F("[WEB] HTTP server started on port "));
  Serial.println(WEB_SERVER_PORT);

  g_webSocket.begin();
  g_webSocket.onEvent(webSocketEvent);
  Serial.print(F("[WEB] WebSocket server started on port "));
  Serial.println(WEBSOCKET_PORT);

  return true;
}

void webUI_handleClient() {
  g_server.handleClient();
  g_webSocket.loop();
}

void webUI_pushUpdates() {
  unsigned long now = millis();

  if (now - g_lastWSUpdate >= WEBSOCKET_UPDATE_INTERVAL_MS) {
    g_lastWSUpdate = now;

    sendSensorUpdate();
    sendSystemStatus();

    static bool wasActive = false;
    bool isActive = adaptive_isCalibrating();

    if (isActive) {
      unsigned long elapsed = now - adaptive_getCalibrationStartTime();
      unsigned long remaining;
      if (elapsed >= (CALIBRATION_TOTAL_SEC * 1000UL)) {
        remaining = 0;
      } else {
        remaining = (CALIBRATION_TOTAL_SEC * 1000UL) - elapsed;
      }

      StaticJsonDocument<128> calibDoc;
      calibDoc["type"] = WS_CALIBRATION_STATUS;
      calibDoc["active"] = true;
      calibDoc["remaining"] = remaining / 1000;
      calibDoc["band"] = adaptive_getCurrentBand();

      char outputActive[128];
      size_t len = serializeJson(calibDoc, outputActive, sizeof(outputActive));
      if (len >= sizeof(outputActive)) {
        Serial.println(F("[WS] WARNING: Calibration active JSON truncated"));
        return;
      }
      g_webSocket.broadcastTXT((const uint8_t*)outputActive, len);
    }

    if (!isActive && wasActive) {
      StaticJsonDocument<128> calibDoc;
      calibDoc["type"] = WS_CALIBRATION_STATUS;
      calibDoc["active"] = false;
      calibDoc["remaining"] = 0;

      char outputInactive[128];
      size_t len = serializeJson(calibDoc, outputInactive, sizeof(outputInactive));
      if (len >= sizeof(outputInactive)) {
        Serial.println(F("[WS] WARNING: Calibration inactive JSON truncated"));
        return;
      }
      g_webSocket.broadcastTXT((const uint8_t*)outputInactive, len);
    }

    wasActive = isActive;
  }
}
