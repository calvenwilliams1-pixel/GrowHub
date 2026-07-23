/*
   web_ui.cpp
   GrowHub32 - Local Web Application Interface Implementation
   Version: 1.3.0
   Revision: Updated CALIBRATION_DURATION_SEC to CALIBRATION_TOTAL_SEC.
             Removed adaptive_updateCalibration() double-call from pushUpdates().
             Added #include "system_state.h" for centralized state access.
             Fixed g_systemState.calibrationActive read outside mutex in sendSensorUpdate().
             Updated UI text for manual override description.
             Updated default values for 88% ceiling and 20-min calibration.

   This serves a single-page application directly from program memory
   to avoid SPIFFS/LittleFS dependency. The HTML, CSS, and JavaScript
   are embedded as raw string literals for maximum reliability.

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
#include <ArduinoJson.h>

static WebServer g_server(WEB_SERVER_PORT);
static WebSocketsServer g_webSocket(WEBSOCKET_PORT);
static unsigned long g_lastWSUpdate = 0;

extern portMUX_TYPE g_stateMux;

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
<title>GrowHub32 v1.3</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box;}
  body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;color:#c9d1d9;min-height:100vh;}
  .header{background:#161b22;padding:12px 16px;border-bottom:1px solid #30363d;position:sticky;top:0;z-index:100;}
  .header h1{font-size:1.2em;color:#58a6ff;}
  .header .status{font-size:0.75em;color:#8b949e;margin-top:2px;}
  .warning-banner{background:#da3633;color:#fff;text-align:center;padding:8px;font-weight:bold;display:none;animation:flash 1s infinite;}
  .warning-banner.active{display:block;}
  @keyframes flash{0%,100%{opacity:1;}50%{opacity:0.5;}}
  .tabs{display:flex;background:#161b22;border-bottom:1px solid #30363d;overflow-x:auto;}
  .tab{padding:10px 16px;font-size:0.85em;color:#8b949e;border:none;background:none;cursor:pointer;white-space:nowrap;border-bottom:2px solid transparent;transition:all 0.2s;}
  .tab.active{color:#58a6ff;border-bottom-color:#58a6ff;}
  .tab-content{display:none;padding:16px;}
  .tab-content.active{display:block;}
  .sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;margin-bottom:16px;}
  .sensor-card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;text-align:center;}
  .sensor-card .label{font-size:0.7em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;}
  .sensor-card .value{font-size:1.8em;font-weight:bold;margin:4px 0;color:#e6edf3;}
  .sensor-card .unit{font-size:0.7em;color:#8b949e;}
  .sensor-card .status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px;}
  .sensor-card .status-dot.ok{background:#3fb950;}
  .sensor-card .status-dot.fault{background:#da3633;}
  .relay-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;margin-bottom:16px;}
  .relay-card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px;text-align:center;}
  .relay-card .name{font-size:0.7em;color:#8b949e;text-transform:uppercase;}
  .relay-card .state{font-size:1em;font-weight:bold;margin:4px 0;}
  .relay-card .state.on{color:#3fb950;}
  .relay-card .state.off{color:#8b949e;}
  .relay-card .locked{color:#da3633;font-size:0.65em;}
  .btn{padding:8px 16px;border:none;border-radius:6px;font-size:0.85em;cursor:pointer;margin:4px;transition:background 0.2s;}
  .btn-on{background:#238636;color:#fff;}
  .btn-on:hover{background:#2ea043;}
  .btn-off{background:#da3633;color:#fff;}
  .btn-off:hover{background:#f85149;}
  .btn-neutral{background:#30363d;color:#c9d1d9;}
  .btn-neutral:hover{background:#484f58;}
  .config-group{margin-bottom:16px;background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;}
  .config-group h3{font-size:0.9em;color:#58a6ff;margin-bottom:10px;}
  .config-row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #21262d;}
  .config-row:last-child{border-bottom:none;}
  .config-row label{font-size:0.8em;}
  .config-row input{width:70px;background:#0d1117;border:1px solid #30363d;border-radius:4px;color:#c9d1d9;padding:4px 8px;font-size:0.85em;text-align:center;}
  .log-area{background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:12px;max-height:300px;overflow-y:auto;font-family:monospace;font-size:0.75em;line-height:1.6;}
  .log-entry{padding:2px 0;}
  .log-entry.warn{color:#d29922;}
  .log-entry.error{color:#da3633;}
  .calibration-panel{text-align:center;padding:20px;}
  .countdown{font-size:3em;font-weight:bold;color:#58a6ff;}
  .sim-result{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-top:10px;text-align:center;}
  .sim-result .time{font-size:1.5em;color:#3fb950;}
  .footer{text-align:center;padding:16px;font-size:0.7em;color:#484f58;}
  .override-panel{display:none;background:#d29922;color:#000;padding:8px;border-radius:6px;margin-bottom:12px;text-align:center;font-weight:bold;}
</style>
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
      <div class="unit">C</div>
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
    <h3>Adaptive Learning</h3>
    <div class="config-row"><label>EMA Weight (0.10-0.50)</label><input type="number" id="emaWeight" value="0.30" step="0.05" min="0.10" max="0.50"></div>
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

<div id="logs" class="tab-content">
  <h3>System Log</h3>
  <div class="log-area" id="logArea">
    <div class="log-entry">Waiting for data...</div>
  </div>
</div>

<div class="footer">GrowHub32 v1.3 | Calvin</div>

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
    ws.close();
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
  }
}

function updateSensors(msg){
  document.getElementById('tempValue').textContent = msg.temp.toFixed(1);
  document.getElementById('humValue').textContent = msg.hum.toFixed(1);
  document.getElementById('co2Value').textContent = msg.co2;
  document.getElementById('fridgeValue').textContent = msg.fridge.toFixed(1);

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
  document.getElementById('simCurrentRH').textContent = msg.hum.toFixed(1);
  document.getElementById('simBand').textContent = 'Band ' + msg.activeBand;
  document.getElementById('simConfidence').textContent = (msg.confidence * 100).toFixed(1) + '%';
  document.getElementById('controlMode').textContent = msg.controlMode || '--';
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
  document.getElementById('assistOn').value = msg.assistOnSec;
  document.getElementById('assistOff').value = msg.assistOffSec;
  document.getElementById('co2High').value = msg.co2HighLimit;
  document.getElementById('co2Low').value = msg.co2LowTarget;
  document.getElementById('co2Emergency').value = msg.co2Emergency;
  document.getElementById('emaWeight').value = msg.emaWeight;
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
  logLines.unshift({time: now, msg: message, level: level});
  if(logLines.length > 100) logLines.pop();

  var logArea = document.getElementById('logArea');
  logArea.innerHTML = logLines.map(function(l){
    return '<div class="log-entry ' + l.level + '">[' + l.time + '] ' + l.msg + '</div>';
  }).join('');
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
  var thresholds = {
    humHoHFloor: parseFloat(document.getElementById('humHoHFloor').value),
    humAssistFloor: parseFloat(document.getElementById('humAssistFloor').value),
    humCeiling: parseFloat(document.getElementById('humCeiling').value),
    assistOnSec: parseInt(document.getElementById('assistOn').value),
    assistOffSec: parseInt(document.getElementById('assistOff').value),
    co2HighLimit: parseInt(document.getElementById('co2High').value),
    co2LowTarget: parseInt(document.getElementById('co2Low').value),
    co2Emergency: parseInt(document.getElementById('co2Emergency').value)
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

function resumeAutomation(){
  sendWS({type: 6, cmd: 'resume_automation'});
  addLog('Automation resumed', 'info');
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

        // Night mode confirmation for loud relays.
        // If night mode is active, user is trying to turn ON a loud relay,
        // and hasn't confirmed yet, send a confirmation request back.
        if (state && !confirmed && relayManager_isRelayLoud(index)) {
          bool nightMode;
          portENTER_CRITICAL(&g_stateMux);
          nightMode = g_systemState.nightModeActive;
          portEXIT_CRITICAL(&g_stateMux);

          if (nightMode) {
            StaticJsonDocument<128> confirmDoc;
            confirmDoc["type"] = 2;  // WS_ALERT            confirmDoc["message"] = "CONFIRM_LOUD_NIGHT";
            confirmDoc["level"] = "warn";
            confirmDoc["relay"] = index;
            char confirmOutput[128];
            serializeJson(confirmDoc, confirmOutput, sizeof(confirmOutput));
            g_webSocket.sendTXT(num, (const uint8_t*)confirmOutput, strlen(confirmOutput));
            return;
          }
        }

        // If compressor is being turned ON and confirmed during night mode,
        // activate the compressor override so continuous enforcement respects it.
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
      else if (msgType == WS_COMMAND && strcmp(cmd, "calibrate_start") == 0) {
        adaptive_startCalibration();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "calibrate_cancel") == 0) {
        adaptive_cancelCalibration();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "resume_automation") == 0) {
        automation_deactivateAllOverrides();
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
        size_t len = serializeJson(responseDoc, response, sizeof(response));
        if (len >= sizeof(response)) {
          Serial.println(F("[WS] WARNING: Simulation response JSON truncated"));
        }
        g_webSocket.sendTXT(num, (const uint8_t*)response, strlen(response));
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
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, strlen(output));
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
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, strlen(output));
}

static void sendConfigUpdate(uint8_t clientNum) {
  AutomationThresholds* t = automation_getThresholds();

  StaticJsonDocument<256> doc;
  doc["type"] = WS_THRESHOLD_UPDATE;
  doc["humHoHFloor"] = t->humHoHFloor;
  doc["humAssistFloor"] = t->humAssistFloor;
  doc["humCeiling"] = t->humCeiling;
  doc["assistOnSec"] = t->assistOnSec;
  doc["assistOffSec"] = t->assistOffSec;
  doc["co2HighLimit"] = t->co2HighLimit;
  doc["co2LowTarget"] = t->co2LowTarget;
  doc["co2Emergency"] = t->co2Emergency;
  doc["emaWeight"] = adaptive_getEMAWeight();

  char output[256];
  size_t len = serializeJson(doc, output, sizeof(output));
  if (len >= sizeof(output)) {
    Serial.println(F("[WS] WARNING: Config update JSON truncated — increase buffer size"));
  }
  g_webSocket.sendTXT(clientNum, (const uint8_t*)output, strlen(output));
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
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, strlen(output));
}

// ============================================================
// Public API
// ============================================================

bool webUI_init() {
  Serial.println(F("[WEB] Initializing web server..."));

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
      }
      g_webSocket.broadcastTXT((const uint8_t*)outputActive, strlen(outputActive));
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
      }
      g_webSocket.broadcastTXT((const uint8_t*)outputInactive, strlen(outputInactive));
    }

    wasActive = isActive;
  }
}
/*
   web_ui.cpp
   GrowHub32 - Local Web Application Interface Implementation
   Version: 1.3.0
   Revision: Updated CALIBRATION_DURATION_SEC to CALIBRATION_TOTAL_SEC.
             Removed adaptive_updateCalibration() double-call from pushUpdates().
             Added #include "system_state.h" for centralized state access.
             Fixed g_systemState.calibrationActive read outside mutex in sendSensorUpdate().
             Updated UI text for manual override description.
             Updated default values for 88% ceiling and 20-min calibration.

   This serves a single-page application directly from program memory
   to avoid SPIFFS/LittleFS dependency. The HTML, CSS, and JavaScript
   are embedded as raw string literals for maximum reliability.

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
#include <ArduinoJson.h>

static WebServer g_server(WEB_SERVER_PORT);
static WebSocketsServer g_webSocket(WEBSOCKET_PORT);
static unsigned long g_lastWSUpdate = 0;

extern portMUX_TYPE g_stateMux;

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
<title>GrowHub32 v1.3</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box;}
  body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;color:#c9d1d9;min-height:100vh;}
  .header{background:#161b22;padding:12px 16px;border-bottom:1px solid #30363d;position:sticky;top:0;z-index:100;}
  .header h1{font-size:1.2em;color:#58a6ff;}
  .header .status{font-size:0.75em;color:#8b949e;margin-top:2px;}
  .warning-banner{background:#da3633;color:#fff;text-align:center;padding:8px;font-weight:bold;display:none;animation:flash 1s infinite;}
  .warning-banner.active{display:block;}
  @keyframes flash{0%,100%{opacity:1;}50%{opacity:0.5;}}
  .tabs{display:flex;background:#161b22;border-bottom:1px solid #30363d;overflow-x:auto;}
  .tab{padding:10px 16px;font-size:0.85em;color:#8b949e;border:none;background:none;cursor:pointer;white-space:nowrap;border-bottom:2px solid transparent;transition:all 0.2s;}
  .tab.active{color:#58a6ff;border-bottom-color:#58a6ff;}
  .tab-content{display:none;padding:16px;}
  .tab-content.active{display:block;}
  .sensor-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;margin-bottom:16px;}
  .sensor-card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;text-align:center;}
  .sensor-card .label{font-size:0.7em;color:#8b949e;text-transform:uppercase;letter-spacing:0.5px;}
  .sensor-card .value{font-size:1.8em;font-weight:bold;margin:4px 0;color:#e6edf3;}
  .sensor-card .unit{font-size:0.7em;color:#8b949e;}
  .sensor-card .status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px;}
  .sensor-card .status-dot.ok{background:#3fb950;}
  .sensor-card .status-dot.fault{background:#da3633;}
  .relay-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px;margin-bottom:16px;}
  .relay-card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px;text-align:center;}
  .relay-card .name{font-size:0.7em;color:#8b949e;text-transform:uppercase;}
  .relay-card .state{font-size:1em;font-weight:bold;margin:4px 0;}
  .relay-card .state.on{color:#3fb950;}
  .relay-card .state.off{color:#8b949e;}
  .relay-card .locked{color:#da3633;font-size:0.65em;}
  .btn{padding:8px 16px;border:none;border-radius:6px;font-size:0.85em;cursor:pointer;margin:4px;transition:background 0.2s;}
  .btn-on{background:#238636;color:#fff;}
  .btn-on:hover{background:#2ea043;}
  .btn-off{background:#da3633;color:#fff;}
  .btn-off:hover{background:#f85149;}
  .btn-neutral{background:#30363d;color:#c9d1d9;}
  .btn-neutral:hover{background:#484f58;}
  .config-group{margin-bottom:16px;background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;}
  .config-group h3{font-size:0.9em;color:#58a6ff;margin-bottom:10px;}
  .config-row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #21262d;}
  .config-row:last-child{border-bottom:none;}
  .config-row label{font-size:0.8em;}
  .config-row input{width:70px;background:#0d1117;border:1px solid #30363d;border-radius:4px;color:#c9d1d9;padding:4px 8px;font-size:0.85em;text-align:center;}
  .log-area{background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:12px;max-height:300px;overflow-y:auto;font-family:monospace;font-size:0.75em;line-height:1.6;}
  .log-entry{padding:2px 0;}
  .log-entry.warn{color:#d29922;}
  .log-entry.error{color:#da3633;}
  .calibration-panel{text-align:center;padding:20px;}
  .countdown{font-size:3em;font-weight:bold;color:#58a6ff;}
  .sim-result{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-top:10px;text-align:center;}
  .sim-result .time{font-size:1.5em;color:#3fb950;}
  .footer{text-align:center;padding:16px;font-size:0.7em;color:#484f58;}
  .override-panel{display:none;background:#d29922;color:#000;padding:8px;border-radius:6px;margin-bottom:12px;text-align:center;font-weight:bold;}
</style>
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
      <div class="unit">C</div>
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
    <h3>Adaptive Learning</h3>
    <div class="config-row"><label>EMA Weight (0.10-0.50)</label><input type="number" id="emaWeight" value="0.30" step="0.05" min="0.10" max="0.50"></div>
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

<div id="logs" class="tab-content">
  <h3>System Log</h3>
  <div class="log-area" id="logArea">
    <div class="log-entry">Waiting for data...</div>
  </div>
</div>

<div class="footer">GrowHub32 v1.3 | Calvin</div>

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
    ws.close();
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
  }
}

function updateSensors(msg){
  document.getElementById('tempValue').textContent = msg.temp.toFixed(1);
  document.getElementById('humValue').textContent = msg.hum.toFixed(1);
  document.getElementById('co2Value').textContent = msg.co2;
  document.getElementById('fridgeValue').textContent = msg.fridge.toFixed(1);

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
  document.getElementById('simCurrentRH').textContent = msg.hum.toFixed(1);
  document.getElementById('simBand').textContent = 'Band ' + msg.activeBand;
  document.getElementById('simConfidence').textContent = (msg.confidence * 100).toFixed(1) + '%';
  document.getElementById('controlMode').textContent = msg.controlMode || '--';
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
  document.getElementById('assistOn').value = msg.assistOnSec;
  document.getElementById('assistOff').value = msg.assistOffSec;
  document.getElementById('co2High').value = msg.co2HighLimit;
  document.getElementById('co2Low').value = msg.co2LowTarget;
  document.getElementById('co2Emergency').value = msg.co2Emergency;
  document.getElementById('emaWeight').value = msg.emaWeight;
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
  logLines.unshift({time: now, msg: message, level: level});
  if(logLines.length > 100) logLines.pop();

  var logArea = document.getElementById('logArea');
  logArea.innerHTML = logLines.map(function(l){
    return '<div class="log-entry ' + l.level + '">[' + l.time + '] ' + l.msg + '</div>';
  }).join('');
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
  var thresholds = {
    humHoHFloor: parseFloat(document.getElementById('humHoHFloor').value),
    humAssistFloor: parseFloat(document.getElementById('humAssistFloor').value),
    humCeiling: parseFloat(document.getElementById('humCeiling').value),
    assistOnSec: parseInt(document.getElementById('assistOn').value),
    assistOffSec: parseInt(document.getElementById('assistOff').value),
    co2HighLimit: parseInt(document.getElementById('co2High').value),
    co2LowTarget: parseInt(document.getElementById('co2Low').value),
    co2Emergency: parseInt(document.getElementById('co2Emergency').value)
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

function resumeAutomation(){
  sendWS({type: 6, cmd: 'resume_automation'});
  addLog('Automation resumed', 'info');
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

        // Night mode confirmation for loud relays.
        // If night mode is active, user is trying to turn ON a loud relay,
        // and hasn't confirmed yet, send a confirmation request back.
        if (state && !confirmed && relayManager_isRelayLoud(index)) {
          bool nightMode;
          portENTER_CRITICAL(&g_stateMux);
          nightMode = g_systemState.nightModeActive;
          portEXIT_CRITICAL(&g_stateMux);

          if (nightMode) {
            StaticJsonDocument<128> confirmDoc;
            confirmDoc["type"] = WS_LOG_MESSAGE;
            confirmDoc["message"] = "CONFIRM_LOUD_NIGHT";
            confirmDoc["level"] = "warn";
            confirmDoc["relay"] = index;
            char confirmOutput[128];
            serializeJson(confirmDoc, confirmOutput, sizeof(confirmOutput));
            g_webSocket.sendTXT(num, (const uint8_t*)confirmOutput, strlen(confirmOutput));
            return;
          }
        }

        // If compressor is being turned ON and confirmed during night mode,
        // activate the compressor override so continuous enforcement respects it.
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
      else if (msgType == WS_COMMAND && strcmp(cmd, "calibrate_start") == 0) {
        adaptive_startCalibration();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "calibrate_cancel") == 0) {
        adaptive_cancelCalibration();
      }
      else if (msgType == WS_COMMAND && strcmp(cmd, "resume_automation") == 0) {
        automation_deactivateAllOverrides();
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
        size_t len = serializeJson(responseDoc, response, sizeof(response));
        if (len >= sizeof(response)) {
          Serial.println(F("[WS] WARNING: Simulation response JSON truncated"));
        }
        g_webSocket.sendTXT(num, (const uint8_t*)response, strlen(response));
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
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, strlen(output));
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
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, strlen(output));
}

static void sendConfigUpdate(uint8_t clientNum) {
  AutomationThresholds* t = automation_getThresholds();

  StaticJsonDocument<256> doc;
  doc["type"] = WS_THRESHOLD_UPDATE;
  doc["humHoHFloor"] = t->humHoHFloor;
  doc["humAssistFloor"] = t->humAssistFloor;
  doc["humCeiling"] = t->humCeiling;
  doc["assistOnSec"] = t->assistOnSec;
  doc["assistOffSec"] = t->assistOffSec;
  doc["co2HighLimit"] = t->co2HighLimit;
  doc["co2LowTarget"] = t->co2LowTarget;
  doc["co2Emergency"] = t->co2Emergency;
  doc["emaWeight"] = adaptive_getEMAWeight();

  char output[256];
  size_t len = serializeJson(doc, output, sizeof(output));
  if (len >= sizeof(output)) {
    Serial.println(F("[WS] WARNING: Config update JSON truncated — increase buffer size"));
  }
  g_webSocket.sendTXT(clientNum, (const uint8_t*)output, strlen(output));
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
  }
  g_webSocket.broadcastTXT((const uint8_t*)output, strlen(output));
}

// ============================================================
// Public API
// ============================================================

bool webUI_init() {
  Serial.println(F("[WEB] Initializing web server..."));

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
      }
      g_webSocket.broadcastTXT((const uint8_t*)outputActive, strlen(outputActive));
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
      }
      g_webSocket.broadcastTXT((const uint8_t*)outputInactive, strlen(outputInactive));
    }

    wasActive = isActive;
  }
}
