/*
   web_ui.cpp
   GrowHub32 - Local Web Application Interface Implementation
   Version: 1.2.3
   Revision: Added scoped manual override activation (only on successful relay
             commands, only for the relevant subsystem). Exposed override state
             via WebSocket for UI display. Added resume automation command.
             Fixed fridge reads to use concurrency-safe accessors.

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
#include "sd_logger.h"
#include <ArduinoJson.h>

static WebServer g_server(WEB_SERVER_PORT);
static WebSocketsServer g_webSocket(WEBSOCKET_PORT);
static unsigned long g_lastWSUpdate = 0;

// External declarations
extern AutomationThresholds* automation_getThresholds();
extern void automation_updateThresholds(const AutomationThresholds*);
extern bool adaptive_startCalibration();
extern bool adaptive_isCalibrating();
extern void adaptive_updateCalibration();
extern float adaptive_projectRecoveryTime(float);
extern void adaptive_setEMAWeight(float);
extern float adaptive_getEMAWeight();
extern unsigned long adaptive_getCalibrationStartTime();
extern bool sdLogger_writeData();

// Forward declarations
static void handleRoot();
static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
static void sendSensorUpdate();
static void sendSystemStatus();

// ============================================================
// EMBEDDED HTML/CSS/JS (Single Page Application)
// ============================================================

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>GrowHub32 v1.2</title>
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
  <button class="tab active" onclick="switchTab('dashboard')">Dashboard</button>
  <button class="tab" onclick="switchTab('controls')">Controls</button>
  <button class="tab" onclick="switchTab('config')">Config</button>
  <button class="tab" onclick="switchTab('calibration')">Calibrate</button>
  <button class="tab" onclick="switchTab('simulation')">Simulate</button>
  <button class="tab" onclick="switchTab('logs')">Logs</button>
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
  </div>
</div>

<div id="controls" class="tab-content">
  <h3>Manual Relay Override</h3>
  <p style="font-size:0.75em;color:#8b949e;">Calibration mode must be OFF to use manual controls.</p>
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
    <div class="config-row"><label>Assist Floor (%)</label><input type="number" id="humAssistFloor" value="75" step="1"></div>
    <div class="config-row"><label>Ceiling (%)</label><input type="number" id="humCeiling" value="95" step="1"></div>
    <div class="config-row"><label>Assist ON (sec)</label><input type="number" id="assistOn" value="5" step="1"></div>
    <div class="config-row"><label>Assist OFF (sec)</label><input type="number" id="assistOff" value="15" step="1"></div>
  </div>
  <div class="config-group">
    <h3>CO2 Thresholds</h3>
    <div class="config-row"><label>High Limit (ppm)</label><input type="number" id="co2High" value="650" step="10"></div>
    <div class="config-row"><label>Low Target (ppm)</label><input type="number" id="co2Low" value="600" step="10"></div>
    <div class="config-row"><label>Emergency (ppm)</label><input type="number" id="co2Emergency" value="800" step="10"></div>
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
    <button class="btn btn-on" id="calibStartBtn" onclick="startCalibration()">Start 15-Minute Calibration</button>
    <button class="btn btn-off" id="calibCancelBtn" onclick="cancelCalibration()" style="display:none;">Cancel Calibration</button>
  </div>
</div>

<div id="simulation" class="tab-content">
  <h3>Recovery Simulation</h3>
  <div class="config-group">
    <div class="config-row"><label>Current RH (%)</label><span id="simCurrentRH">--</span></div>
    <div class="config-row"><label>Target RH (%)</label><input type="number" id="simTargetRH" value="95" step="1"></div>
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

<div class="footer">GrowHub32 v1.2 | Calvin</div>

<script>
var ws;
var logLines = [];

function connectWS(){
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = function(){
    document.getElementById('connectionStatus').textContent = 'Connected | ' + location.hostname;
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
    setTimeout(connectWS, 3000);
  };
  ws.onerror = function(){
    ws.close();
  };
}

function handleMessage(msg){
  switch(msg.type){
    case 0: updateSensors(msg); break;
    case 1: updateRelays(msg); updateOverrideStatus(msg); break;
    case 2: addLog(msg.message, msg.level || 'warn'); break;
    case 4: updateCalibration(msg); break;
    case 5: addLog(msg.message, msg.level || 'info'); break;
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

function switchTab(tabId){
  document.querySelectorAll('.tab').forEach(function(t){ t.classList.remove('active'); });
  document.querySelectorAll('.tab-content').forEach(function(c){ c.classList.remove('active'); });
  event.target.classList.add('active');
  document.getElementById(tabId).classList.add('active');
}

function relayCmd(index, state){
  ws.send(JSON.stringify({type: 6, cmd: 'relay', index: index, state: state}));
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
  ws.send(JSON.stringify({type: 6, cmd: 'thresholds', data: thresholds}));

  var emaWeight = parseFloat(document.getElementById('emaWeight').value);
  ws.send(JSON.stringify({type: 6, cmd: 'ema', weight: emaWeight}));

  addLog('Settings saved!', 'info');
}

function startCalibration(){
  ws.send(JSON.stringify({type: 6, cmd: 'calibrate_start'}));
}

function cancelCalibration(){
  ws.send(JSON.stringify({type: 6, cmd: 'calibrate_cancel'}));
}

function runSimulation(){
  var target = parseFloat(document.getElementById('simTargetRH').value);
  var current = parseFloat(document.getElementById('simCurrentRH').textContent);
  ws.send(JSON.stringify({type: 6, cmd: 'simulate', current: current, target: target}));
}

function resumeAutomation(){
  ws.send(JSON.stringify({type: 6, cmd: 'resume_automation'}));
  addLog('Automation resumed', 'info');
}

ws && ws.addEventListener('message', function(e){
  try{
    var msg = JSON.parse(e.data);
    if(msg.type === 99 && msg.simResult){
      document.getElementById('simResult').textContent = msg.simResult;
    }
  }catch(e){}
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
  g_server.send(200, "text/html", INDEX_HTML);
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
      break;

    case WStype_TEXT: {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, payload, length);

      if (error) {
        Serial.print(F("[WS] JSON parse error: "));
        Serial.println(error.c_str());
        return;
      }

      uint8_t msgType = doc["type"] | 0;
      String cmd = doc["cmd"] | "";

      if (msgType == WS_COMMAND && cmd == "relay") {
        uint8_t index = doc["index"] | 0;
        bool state = doc["state"] | false;
        if (!g_systemState.calibrationActive) {
          if (relayManager_setRelay(index, state)) {
            // Activate override only for the relevant subsystem and only on success.
            // Compressor has no competing automation loop, so no override needed.
            if (index == RELAY_HOH || index == RELAY_AIR_ASSIST) {
              automation_activateHumidityOverride();
            } else if (index == RELAY_EXHAUST) {
              automation_activateCO2Override();
            }
            Serial.print(F("[WS] Relay "));
            Serial.print(index);
            Serial.print(F(" -> "));
            Serial.println(state ? "ON" : "OFF");
          }
        }
      }
      else if (msgType == WS_COMMAND && cmd == "thresholds") {
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
      else if (msgType == WS_COMMAND && cmd == "ema") {
        float weight = doc["weight"] | DEFAULT_EMA_WEIGHT;
        adaptive_setEMAWeight(weight);
      }
      else if (msgType == WS_COMMAND && cmd == "calibrate_start") {
        adaptive_startCalibration();
      }
      else if (msgType == WS_COMMAND && cmd == "calibrate_cancel") {
        adaptive_cancelCalibration();
      }
      else if (msgType == WS_COMMAND && cmd == "resume_automation") {
        automation_deactivateAllOverrides();
      }
      else if (msgType == WS_COMMAND && cmd == "simulate") {
        float current = doc["current"] | 0.0f;
        float target = doc["target"] | 95.0f;
        float delta = target - current;
        if (delta > 0) {
          float recoveryTime = adaptive_projectRecoveryTime(delta);
          String simResult = String(recoveryTime, 0) + " seconds (" +
                            String(recoveryTime / 60.0f, 1) + " minutes)";

          StaticJsonDocument<128> responseDoc;
          responseDoc["type"] = 99;
          responseDoc["simResult"] = simResult;
          String response;
          serializeJson(responseDoc, response);
          g_webSocket.sendTXT(num, response);
        }
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
  StaticJsonDocument<512> doc;
  doc["type"] = WS_SENSOR_UPDATE;
  doc["temp"] = g_systemState.currentTemp;
  doc["hum"] = g_systemState.currentHumidity;
  doc["co2"] = g_systemState.currentCO2;
  doc["fridge"] = network_getFridgeTemp();
  doc["tempFault"] = g_systemState.tempSensorFault;
  doc["humFault"] = g_systemState.humiditySensorFault;
  doc["co2Fault"] = g_systemState.co2SensorFault;
  doc["nightMode"] = g_systemState.nightModeActive;
  doc["wifiConnected"] = g_systemState.wifiConnected;
  doc["apMode"] = g_systemState.apModeActive;
  doc["rtcTime"] = rtc_getTimeString();
  doc["fridgeLost"] = network_isFridgeHeartbeatLost();
  doc["activeBand"] = adaptive_getCurrentBand();

  BandProfile* profile = adaptive_getActiveProfile();
  doc["confidence"] = profile ? profile->confidenceScore : 0.0f;

  String output;
  serializeJson(doc, output);
  g_webSocket.broadcastTXT(output);
}

static void sendSystemStatus() {
  StaticJsonDocument<256> doc;
  doc["type"] = WS_RELAY_STATE;
  doc["hoh"] = g_systemState.hoHActive;
  doc["assist"] = g_systemState.airAssistActive;
  doc["fan"] = g_systemState.exhaustFanActive;
  doc["compressor"] = g_systemState.compressorActive;
  doc["compressorLocked"] = g_relays[RELAY_COMPRESSOR].cooldownLocked;
  doc["humOverride"] = automation_isHumidityOverrideActive();
  doc["humOverrideRemaining"] = automation_getHumidityOverrideRemaining() / 1000;
  doc["co2Override"] = automation_isCO2OverrideActive();
  doc["co2OverrideRemaining"] = automation_getCO2OverrideRemaining() / 1000;

  String output;
  serializeJson(doc, output);
  g_webSocket.broadcastTXT(output);
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

    if (adaptive_isCalibrating()) {
      adaptive_updateCalibration();

      unsigned long elapsed = now - adaptive_getCalibrationStartTime();
      unsigned long remaining = (CALIBRATION_DURATION_SEC * 1000UL) - elapsed;

      StaticJsonDocument<128> calibDoc;
      calibDoc["type"] = WS_CALIBRATION_STATUS;
      calibDoc["active"] = true;
      calibDoc["remaining"] = remaining / 1000;
      calibDoc["band"] = adaptive_getCurrentBand();

      String output;
      serializeJson(calibDoc, output);
      g_webSocket.broadcastTXT(output);
    } else {
      static bool wasActive = false;
      if (wasActive) {
        StaticJsonDocument<128> calibDoc;
        calibDoc["type"] = WS_CALIBRATION_STATUS;
        calibDoc["active"] = false;
        calibDoc["remaining"] = 0;

        String output;
        serializeJson(calibDoc, output);
        g_webSocket.broadcastTXT(output);
      }
      wasActive = adaptive_isCalibrating();
    }
  }

  g_server.handleClient();
  g_webSocket.loop();
}
