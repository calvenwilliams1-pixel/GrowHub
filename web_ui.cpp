/*
   web_ui.cpp
   GrowHub32 - Local Web Application Interface Implementation
   Version: 1.4.0
   Revision: Updated CALIBRATION_DURATION_SEC to CALIBRATION_TOTAL_SEC.
             Removed adaptive_updateCalibration() double-call from pushUpdates().
             Added #include "system_state.h" for centralized state access.
             Fixed g_systemState.calibrationActive read outside mutex in sendSensorUpdate().
             Updated UI text for manual override description.
             Updated default values for 88% ceiling and 20-min calibration.

   This serves a single-page application from program memory.
   Chart.js is served from LittleFS for cache efficiency (v1.4).

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
#include <LittleFS.h>

// ============================================================
// UPDATE VERSION HERE WHEN BUMPING FIRMWARE
// ============================================================
#define WEB_UI_VERSION "1.4.0"

static WebServer g_server(WEB_SERVER_PORT);
static WebSocketsServer g_webSocket(WEBSOCKET_PORT);
static unsigned long g_lastWSUpdate = 0;

extern portMUX_TYPE g_stateMux;
extern RuntimeCache g_runtimeCache;

// Forward declarations
static void handleRoot();
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
static void sendSensorUpdate();
static void sendSystemStatus();
static void sendConfigUpdate(uint8_t clientNum);
static void sendCalibrationUpdate();

// ============================================================
// EMBEDDED HTML/CSS/JS (Single Page Application)
// ============================================================

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>GrowHub32 v1.4</title>  <!-- bump WEB_UI_VERSION above -->
<style>
  *{margin:0;padding:0;box-sizing:border-box;}
  body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:linear-gradient(180deg,#0d1117 0%,#111827 100%);color:#c9d1d9;min-height:100vh;}
  .sticky-header-wrapper{position:sticky;top:0;z-index:100;background:#161b22;}
  .header{padding:14px 18px;border-bottom:1px solid #30363d;}
  .header h1{font-size:1.3em;color:#58a6ff;}
  .header .status{font-size:0.75em;color:#8b949e;margin-top:3px;}
  .warning-banner{color:#fff;text-align:center;padding:10px 16px;font-weight:600;display:none;border-bottom:1px solid #f85149;}
  .warning-banner.active{display:flex;align-items:center;justify-content:center;gap:8px;animation:pulse-danger 2s infinite;}
  @keyframes pulse-danger{0%{background-color:#da3633;}50%{background-color:#8e1519;}100%{background-color:#da3633;}}
  .tabs{display:flex;background:rgba(22,27,34,0.95);backdrop-filter:blur(5px);border-bottom:1px solid #30363d;overflow-x:auto;}
  .tab{padding:14px 20px;font-size:0.9em;color:#8b949e;border:none;background:none;cursor:pointer;white-space:nowrap;border-bottom:2px solid transparent;transition:all 0.2s;}
  .tab.active{color:#58a6ff;border-bottom-color:#58a6ff;}
  .tab-content{display:none;padding:16px;}
  .tab-content.active{display:block;}
  .sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:14px;margin-bottom:18px;}
  .sensor-card{background:#161b22;border:1px solid #3d444d;border-radius:12px;padding:18px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.5);transition:box-shadow 0.2s ease;}
  .sensor-card .label{font-size:0.7em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;}
  .sensor-card .value{font-size:2.2em;font-weight:700;margin:8px 0;color:#ffffff;line-height:1;font-variant-numeric:tabular-nums;min-height:1.2em;}
  .sensor-card .unit{font-size:0.6em;color:#8b949e;font-weight:500;text-transform:uppercase;letter-spacing:0.5px;}
  .sensor-card .status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;}
  .sensor-card .status-dot.ok{background:#3fb950;box-shadow:0 0 6px rgba(63,185,80,0.4);}
  .sensor-card .status-dot.fault{background:#da3633;box-shadow:0 0 6px rgba(218,54,51,0.4);}
  .relay-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px;margin-bottom:18px;}
  .relay-card{background:#161b22;border:1px solid #3d444d;border-radius:12px;padding:14px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.5);}
  .relay-card:has(.state.on){border-color:#3fb950;box-shadow:0 0 16px rgba(63,185,80,0.25);}
  .relay-card .name{font-size:0.7em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;}
  .relay-card .state{font-size:1.1em;font-weight:bold;margin:6px 0;}
  .relay-card .state.on{color:#3fb950;}
  .relay-card .state.off{color:#8b949e;}
  .relay-card .locked{color:#d29922;font-size:0.7em;margin-top:4px;}
  .btn{padding:12px 20px;border:none;border-radius:8px;font-size:0.95em;cursor:pointer;margin:4px;transition:all 0.15s ease-in-out;font-weight:500;}
  .btn:active{transform:scale(0.96);filter:brightness(0.9);}
  .btn:focus{outline:2px solid #58a6ff;outline-offset:2px;}
  .btn-on{background:#238636;color:#fff;}
  .btn-on:hover{background:#2ea043;}
  .btn-off{background:#21262d;border:1px solid #30363d;color:#c9d1d9;}
  .btn-off:hover{background:#30363d;}
  .btn-neutral{background:#30363d;color:#c9d1d9;}
  .btn-neutral:hover{background:#484f58;}
  .config-group{margin-bottom:18px;background:#161b22;border:1px solid #3d444d;border-radius:12px;padding:16px;}
  .config-group h3{font-size:0.95em;color:#58a6ff;margin-bottom:12px;border-left:4px solid #58a6ff;padding-left:10px;}
  .config-row{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid #21262d;}
  .config-row:last-child{border-bottom:none;}
  .config-row label{font-size:0.85em;color:#c9d1d9;font-weight:500;}
  .config-row input{width:90px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:8px 12px;font-size:0.95em;text-align:right;font-variant-numeric:tabular-nums;}
  .config-row input:focus{outline:2px solid #58a6ff;outline-offset:2px;}
  .config-row input:invalid{border-color:#da3633;color:#da3633;box-shadow:0 0 8px rgba(218,54,51,0.4);}
  .log-area{background:#0d1117;border:1px solid #3d444d;border-radius:12px;padding:14px;max-height:300px;overflow-y:auto;font-family:monospace;font-size:0.75em;line-height:1.6;}
  .log-entry{padding:3px 0;}
  .log-entry.warn{color:#d29922;}
  .log-entry.error{color:#da3633;}
  .calibration-panel{text-align:center;padding:24px;}
  .countdown{font-size:3em;font-weight:bold;color:#58a6ff;}
  .sim-result{background:#161b22;border:1px solid #3d444d;border-radius:12px;padding:16px;margin-top:12px;text-align:center;}
  .sim-result .time{font-size:1.5em;color:#3fb950;}
  .footer{text-align:center;padding:18px;font-size:0.7em;color:#484f58;}
  .override-panel{display:none;background:#3a2a1a;color:#d29922;padding:10px;border-radius:8px;margin-bottom:14px;text-align:center;font-weight:bold;border:1px solid #d29922;}
  .status-pill{display:inline-block;padding:4px 10px;border-radius:999px;font-size:0.7em;font-weight:500;margin:2px 4px;}
  .status-pill.good{background:#1a3a1a;color:#3fb950;border:1px solid #3fb950;}
  .status-pill.warn{background:#3a2a1a;color:#d29922;border:1px solid #d29922;}
  .sticky-save-container{position:sticky;bottom:16px;background:rgba(13,17,23,0.95);padding:12px;border-radius:8px;border:1px solid #30363d;text-align:center;margin-top:16px;box-shadow:0 -4px 12px rgba(0,0,0,0.4);}
  ::-webkit-scrollbar{width:6px;height:6px;}
  ::-webkit-scrollbar-track{background:transparent;}
  ::-webkit-scrollbar-thumb{background:#3d444d;border-radius:10px;}
  ::-webkit-scrollbar-thumb:hover{background:#58a6ff;}
</style>
<script src="/chart-4.4.0.min.js"></script>
</head>
<body>
<div class="header">
  <h1>GrowHub32</h1>
  <div class="status" id="connectionStatus">Connecting...</div>
</div>

<div class="warning-banner" id="warningBanner">Warning: Sensor Fault - System Running Last Known Values</div>

<div class="tabs">
  <button class="tab active" onclick="switchTab(this, 'dashboard')">Dashboard</button>
  <button class="tab" onclick="switchTab(this, 'controls')">Controls</button>
  <button class="tab" onclick="switchTab(this, 'config')">Config</button>
  <button class="tab" onclick="switchTab(this, 'calibration')">Calibrate</button>
  <button class="tab" onclick="switchTab(this, 'simulation')">Simulate</button>
  <button class="tab" onclick="switchTab(this, 'graphs')">Graphs</button>
  <button class="tab" onclick="switchTab(this, 'logs')">Logs</button>
</div>

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
  <div class="relay-grid">
    <div class="relay-card"><div class="name">Humidifier</div><div class="state off" id="hohState">OFF</div></div>
    <div class="relay-card"><div class="name">Air Assist</div><div class="state off" id="assistState">OFF</div></div>
    <div class="relay-card"><div class="name">Exhaust Fan</div><div class="state off" id="fanState">OFF</div></div>
    <div class="relay-card"><div class="name">Compressor</div><div class="state off" id="compState">OFF</div><div class="locked" id="compLock"></div></div>
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

<div id="controls" class="tab-content">
  <h3>Manual Relay Override</h3>
  <p style="font-size:0.75em;color:#8b949e;">Calibration mode must be OFF to use manual controls. Manual commands pause automation. Safety interlocks (such as compressor cooldown) remain active.</p>
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
    <h3>Relay Mapping</h3>
    <p style="font-size:0.7em;color:#8b949e;margin-bottom:8px;">Changes take effect immediately. GPIO 0-3,5,12,15,18-23 are blocked.</p>
    <div class="config-row"><label>HOH Humidifier Pin</label><input type="number" id="pinHOH" value="13" min="0" max="39"></div>
    <div class="config-row"><label>Air Assist Pin</label><input type="number" id="pinAirAssist" value="26" min="0" max="39"></div>
    <div class="config-row"><label>Exhaust Fan Pin</label><input type="number" id="pinExhaust" value="14" min="0" max="39"></div>
    <div class="config-row"><label>Compressor Pin</label><input type="number" id="pinCompressor" value="27" min="0" max="39"></div>
    <button class="btn btn-on" onclick="saveRelayMapping()">Apply Relay Mapping</button>
  </div>
  <button class="btn btn-on" onclick="saveThresholds()">Save All Settings</button>
</div>

<div id="calibration" class="tab-content">
  <div class="calibration-panel">
    <h3>Calibration Status</h3>
    <div class="countdown" id="calibCountdown">--</div>
    <p id="calibStatus" style="color:#8b949e;">Not active</p>
    <button class="btn btn-on" id="calibStartBtn" onclick="startCalibration()">Start 20-Minute Calibration</button>
    <button class="btn btn-off" id="calibCancelBtn" onclick="cancelCalibration()" style="display:none;">Cancel Calibration</button>
  </div>
</div>

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

<div id="logs" class="tab-content">
  <h3>System Log</h3>
  <div class="log-area" id="logArea">
    <div class="log-entry">Waiting for data...</div>
  </div>
</div>

<div class="footer">GrowHub32 v1.4 | Calvin</div>

<script>
var ws;
var logLines = [];
var reconnectDelay = 3000;

function sendWS(data){
  if(ws && ws.readyState === WebSocket.OPEN){
    ws.send(JSON.stringify(data));
  }
}

function connectWS(){
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = function(){
    document.getElementById('connectionStatus').textContent = 'Connected | ' + location.hostname;
    reconnectDelay = 3000;
    initGraph();
    requestHistorical();
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
    case 1: updateRelays(msg); updateOverrideStatus(msg); break;
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
  document.getElementById('co2Value').textContent = (msg.co2 != null) ? msg.co2 : '--';
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
}

function updateRelays(msg){
  var hoh = document.getElementById('hohState');
  hoh.textContent = msg.hoh ? 'ON' : 'OFF';
  hoh.className = 'state ' + (msg.hoh ? 'on' : 'off');

  var assist = document.getElementById('assistState');
  assist.textContent = msg.assist ? 'ON' : 'OFF';
  assist.className = 'state ' + (msg.assist ? 'on' : 'off');

  var fan = document.getElementById('fanState');
  fan.textContent = msg.fan ? 'ON' : 'OFF';
  fan.className = 'state ' + (msg.fan ? 'on' : 'off');

  var comp = document.getElementById('compState');
  comp.textContent = msg.compressor ? 'ON' : 'OFF';
  comp.className = 'state ' + (msg.compressor ? 'on' : 'off');

  document.getElementById('compLock').textContent = msg.compressorLocked ? '(COOLDOWN)' : '';
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
  document.getElementById('pinHOH').value = msg.pinHOH;
  document.getElementById('pinAirAssist').value = msg.pinAirAssist;
  document.getElementById('pinExhaust').value = msg.pinExhaust;
  document.getElementById('pinCompressor').value = msg.pinCompressor;
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
  var now = new Date().toLocaleTimeString();
  var logArea = document.getElementById('logArea');
  var div = document.createElement('div');
  div.className = 'log-entry ' + (level || 'info');
  div.textContent = '[' + now + '] ' + message;
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

  if (isNaN(hohFloor) || isNaN(assistFloor) || isNaN(ceiling) || isNaN(exhaustOn)) {
    addLog('Invalid humidity threshold value', 'warn');
    return;
  }
  if (isNaN(assistOn) || isNaN(assistOff)) {
    addLog('Invalid assist timing value', 'warn');
    return;
  }
  if (isNaN(co2High) || isNaN(co2Low) || isNaN(co2Emer)) {
    addLog('Invalid CO2 threshold value', 'warn');
    return;
  }

  var thresholds = {
    humHoHFloor: hohFloor,
    humAssistFloor: assistFloor,
    humCeiling: ceiling,
    humExhaustOn: exhaustOn,
    assistOnSec: assistOn,
    assistOffSec: assistOff,
    co2HighLimit: co2High,
    co2LowTarget: co2Low,
    co2Emergency: co2Emer
  };
  sendWS({type: 6, cmd: 'thresholds', data: thresholds});

  var emaWeight = parseFloat(document.getElementById('emaWeight').value);
  sendWS({type: 6, cmd: 'ema', weight: emaWeight});

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
  var pins = {
    pinHOH: parseInt(document.getElementById('pinHOH').value, 10),
    pinAirAssist: parseInt(document.getElementById('pinAirAssist').value, 10),
    pinExhaust: parseInt(document.getElementById('pinExhaust').value, 10),
    pinCompressor: parseInt(document.getElementById('pinCompressor').value, 10)
  };
  sendWS({type: 6, cmd: 'relay_mapping', data: pins});
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
var liveUpdatePending = false;

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
  if (now - graphLastRequestTime < 5000) return;
  graphLastRequestTime = now;
  graphRequestId = (graphRequestId + 1) & 0xFFFF;
  var start = Math.floor(now / 1000) - graphRange;
  sendWS({type: 100, sensor: graphSensor, start: start, end: Math.floor(now / 1000), max: 350, rid: graphRequestId});
}

function feedLiveGraph(sensor, value) {
  var now = Math.floor(Date.now() / 1000);
  liveBuffers[sensor].push({x: now, y: value});
  if (liveBuffers[sensor].length > GRAPH_MAX_LIVE) {
    liveBuffers[sensor] = liveBuffers[sensor].slice(-GRAPH_MAX_LIVE);
  }
  if (graphChart && sensor === graphSensor && !liveUpdatePending) {
    liveUpdatePending = true;
    requestAnimationFrame(function() {
      liveUpdatePending = false;
      if (!graphChart) return;
      graphChart.data.datasets[0].data = liveBuffers[sensor];
      graphChart.update('none');
    });
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
        // Sanity: reject epochs before 2020 to prevent 1970 DoS loop
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

        if (index >= RELAY_COUNT) {
          Serial.print(F("[WS] Invalid relay index: "));
          Serial.println(index);
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
            automation_activateHumidityOverride();
          } else if (index == RELAY_EXHAUST) {
            automation_activateCO2Override();
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
      else if (msgType == WS_COMMAND && strcmp(cmd, "thresholds") == 0) {
        AutomationThresholds* thresholds = automation_getThresholds();
        AutomationThresholds newThresholds = *thresholds;

        newThresholds.humHoHFloor = doc["data"]["humHoHFloor"] | thresholds->humHoHFloor;
        newThresholds.humAssistFloor = doc["data"]["humAssistFloor"] | thresholds->humAssistFloor;
        newThresholds.humCeiling = doc["data"]["humCeiling"] | thresholds->humCeiling;
        newThresholds.humExhaustOn = doc["data"]["humExhaustOn"] | thresholds->humExhaustOn;
        newThresholds.assistOnSec = doc["data"]["assistOnSec"] | thresholds->assistOnSec;
        newThresholds.assistOffSec = doc["data"]["assistOffSec"] | thresholds->assistOffSec;
        newThresholds.co2HighLimit = doc["data"]["co2HighLimit"] | thresholds->co2HighLimit;
        newThresholds.co2LowTarget = doc["data"]["co2LowTarget"] | thresholds->co2LowTarget;
        newThresholds.co2Emergency = doc["data"]["co2Emergency"] | thresholds->co2Emergency;

        automation_updateThresholds(&newThresholds);
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "ema") == 0) {
        float weight = doc["weight"] | DEFAULT_EMA_WEIGHT;
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
      else if (msgType == WS_COMMAND && strcmp(cmd, "relay_mapping") == 0) {
        const RelayMapping* current = relayManager_getMapping();
        RelayMapping newMapping;
        newMapping.magic = RELAY_MAPPING_MAGIC;
        newMapping.pinHOH = doc["data"]["pinHOH"] | current->pinHOH;
        newMapping.pinAirAssist = doc["data"]["pinAirAssist"] | current->pinAirAssist;
        newMapping.pinExhaust = doc["data"]["pinExhaust"] | current->pinExhaust;
        newMapping.pinCompressor = doc["data"]["pinCompressor"] | current->pinCompressor;
        memset(newMapping.reserved, 0, sizeof(newMapping.reserved));

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
          responseDoc["message"] = "Invalid relay mapping — check pins";
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

  StaticJsonDocument<768> doc;
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

  StaticJsonDocument<256> doc;
  doc["type"] = WS_RELAY_STATE;
  doc["hoh"] = hoh;
  doc["assist"] = assist;
  doc["fan"] = fan;
  doc["compressor"] = compressor;
  doc["compressorLocked"] = relayManager_isCompressorCooldownActive();
  doc["humOverride"] = automation_isHumidityOverrideActive();
  doc["humOverrideRemaining"] = automation_getHumidityOverrideRemaining() / 1000;
  doc["co2Override"] = automation_isCO2OverrideActive();
  doc["co2OverrideRemaining"] = automation_getCO2OverrideRemaining() / 1000;

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
  doc["pinHOH"] = mapping->pinHOH;
  doc["pinAirAssist"] = mapping->pinAirAssist;
  doc["pinExhaust"] = mapping->pinExhaust;
  doc["pinCompressor"] = mapping->pinCompressor;
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
// Public API
// ============================================================

bool webUI_init() {
  Serial.println(F("[WEB] Initializing web server..."));

  g_server.on("/chart-4.4.0.min.js", []() {
    if (LittleFS.exists("/chart-4.4.0.min.js")) {
      g_server.sendHeader("Cache-Control", "max-age=31536000, immutable");
      File f = LittleFS.open("/chart-4.4.0.min.js", "r");
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
