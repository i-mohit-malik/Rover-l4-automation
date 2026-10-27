#include "WebHMI.h"
#include "../core/Logger.h"
#include <Update.h>

// ---------------------------------------------------------------------------------------------
// The web page. %TITLE%, %MAXSPD% and %FW% are filled in by _buildPage().
// ---------------------------------------------------------------------------------------------
static const char kPage[] PROGMEM = R"rawpage(<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'><title>%TITLE%</title>
<style>
:root{--bg:#0b0f17;--card:#121a27;--muted:#9aa4b2;--text:#e7edf5;--acc:#4ea1ff;--warn:#ffb020;}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;}
.wrap{max-width:980px;margin:0 auto;padding:16px;}
.top{display:flex;align-items:center;justify-content:space-between;gap:12px;}
.title{font-size:18px;font-weight:700;}
.pill{font-size:12px;color:var(--muted);padding:6px 10px;border:1px solid rgba(255,255,255,.08);border-radius:999px;}
.card{background:var(--card);border:1px solid rgba(255,255,255,.06);border-radius:16px;padding:14px;margin-top:12px;}
.label{font-size:12px;color:var(--muted);}
.value{font-size:16px;font-weight:650;}
.sectionTitle{font-size:13px;color:var(--muted);margin:0 0 8px;}
.hidden{display:none;}
button{appearance:none;border:0;border-radius:14px;font-weight:650;cursor:pointer;color:var(--text);
 background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.09);}
.btnTiny{padding:10px 12px;font-size:13px;}
.stop{background:linear-gradient(180deg,rgba(255,90,95,.95),rgba(255,90,95,.75));color:#190406;}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,72px);gap:10px;
 justify-content:center;margin:6px 0 10px;touch-action:none;user-select:none;-webkit-user-select:none;}
.pad button{font-size:24px;touch-action:none;}
.pad button.on{outline:3px solid var(--acc);}
input[type=range]{width:100%;}
.table{width:100%;border-collapse:collapse;font-size:13px;}
.table td{padding:8px 10px;border-bottom:1px solid rgba(255,255,255,.06);}
.k{color:var(--muted);}
canvas{display:block;margin:0 auto;max-width:100%;}
.badge{display:block;text-align:center;font-weight:700;padding:10px;border-radius:12px;margin:4px 0 10px;}
.b-idle{background:#334155;color:#cbd5e1;}.b-run{background:#16a34a;color:#fff;}
.b-pause{background:#d97706;color:#fff;}.b-stop{background:#dc2626;color:#fff;}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px;}.row2 button{padding:14px;font-size:15px;}
.go{background:linear-gradient(180deg,rgba(16,185,129,.95),rgba(16,185,129,.75));color:#03140d;}
.lost{background:#7f1d1d;border:1px solid #ef4444;text-align:center;font-weight:700;}
a{color:var(--acc);}
</style></head><body><div class='wrap'>
<div class='top'><div class='title'>%TITLE%</div><div class='pill' id='conn'>-</div>
 <button id='devBtn' class='btnTiny'>Dev</button></div>
<div class='card lost hidden' id='lost'>&#9888; No connection to the box - values are frozen, a run stops after 3 s</div>
<div class='card'><div class='label'>State</div><div class='value' id='state'>-</div></div>
<div class='card'><div class='sectionTitle'>Autonomous run (pole row, turn right)</div>
 <div class='badge b-idle' id='runBadge'>-</div>
 <div class='row2'><button class='go' onclick="cmd('/run/start')">&#9654; Start run</button><button class='stop' onclick="cmd('/run/stop')">&#9632; Stop run</button></div>
 <div class='row2'><button id='safBtn' onclick="cmd('/run/safety')">Safety stops</button><button onclick="cmd('/imu/save')">Save IMU calibration</button></div>
 <table class='table' id='runTable'></table>
 <div class='label' id='msg'></div><div class='label'>%FW%</div></div>
<div class='card hidden' id='driveCard'><div class='sectionTitle'>Drive (hold a key; a key also ends an autonomous run)</div>
 <div class='pad'>
  <span></span><button data-d='f'>&#9650;</button><span></span>
  <button data-d='l'>&#9664;</button><button class='stop' data-d='s'>&#9632;</button><button data-d='r'>&#9654;</button>
  <span></span><button data-d='b'>&#9660;</button><span></span>
 </div>
 <div class='label'>Speed <span id='spdVal'></span></div><input type='range' id='spd' min='0.05' step='0.05'></div>
<div class='card'><div class='sectionTitle'>Surroundings (LiDAR top view, front = up, rings 1 m)</div>
 <canvas id='envCv' width='280' height='280'></canvas><div class='label' id='lidarInfo'>-</div>
 <table class='table' id='usTable'></table></div>
<div class='card'><div class='sectionTitle'>IMU / GPS</div><table class='table' id='navTable'></table></div>
<div id='devRoot' class='hidden'>
 <div class='card'><div class='sectionTitle'>Values</div><table class='table' id='valTable'></table></div>
 <div class='card'><div class='sectionTitle'>Loop time (box)</div>
  <table class='table'><tr><td class='k'>min / avg / max</td><td id='cycle'>-</td></tr></table></div>
 <div class='card'><a href='/log' style='color:var(--acc)'>Timeline log (download, mark, clear)</a></div>
</div></div>
<script>
const MAX_SPEED=%MAXSPD%, ENV_MAX_CM=300;
const $=id=>document.getElementById(id);
const esc=s=>(''+s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');

// ---- run card buttons: the box queues them, the result shows in the message line
function cmd(p){fetch(p).catch(()=>{});}

// ---- "Dev" view toggle
let dev=false;
$('devBtn').onclick=()=>{dev=!dev;$('devRoot').classList.toggle('hidden',!dev);$('devBtn').textContent=dev?'Hide':'Dev';};

// ---- drive pad: send while a key is held (every 150 ms), stop on release / hide / blur
let drv=null,drvTimer=null,drvBtn=null;
// Requests can overtake each other on Wi-Fi. Each command carries this page's random id and a
// rising number; the box ignores anything older than the last command it accepted.
const PAGE_ID=Math.floor(Math.random()*4e9)+1;let seq=0;
function drvSend(l,r){seq++;fetch('/drive?l='+l.toFixed(2)+'&r='+r.toFixed(2)+'&id='+PAGE_ID+'&n='+seq).catch(()=>{});}
function drvClear(){clearInterval(drvTimer);drvTimer=null;drv=null;if(drvBtn){drvBtn.classList.remove('on');drvBtn=null;}}
function drvStop(){drvClear();drvSend(0,0);}
function drvStart(b){
  if(b.dataset.d==='s'){drvStop();cmd('/run/stop');return;}
  const s=parseFloat($('spd').value);
  const m={f:[s,s],b:[-s,-s],l:[-s,s],r:[s,-s]}[b.dataset.d];
  drvClear();  // switching keys: no stop in between, the new command follows directly
  drv=m;drvBtn=b;b.classList.add('on');drvSend(m[0],m[1]);
  drvTimer=setInterval(()=>drvSend(drv[0],drv[1]),150);
}
if(MAX_SPEED>0){
  $('driveCard').classList.remove('hidden');
  const sp=$('spd');sp.max=MAX_SPEED;sp.value=MAX_SPEED;
  const show=()=>{$('spdVal').textContent=parseFloat(sp.value).toFixed(2);};
  sp.addEventListener('input',show);show();
  document.querySelectorAll('.pad button').forEach(b=>{
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);drvStart(b);});
    ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>{if(drvBtn===b)drvStop();}));
    b.addEventListener('contextmenu',e=>e.preventDefault());
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&drvBtn)drvStop();});
  window.addEventListener('blur',()=>{if(drvBtn)drvStop();});
}

// ---- LiDAR top view: line through neighbouring points, broken at "no return" and at jumps
function drawEnv(s){
  const cv=$('envCv'),g=cv.getContext('2d'),W=cv.width,c=W/2,k=(c-12)/ENV_MAX_CM;
  g.clearRect(0,0,W,W);
  g.strokeStyle='rgba(255,255,255,.12)';g.lineWidth=1;
  for(let m=100;m<=ENV_MAX_CM;m+=100){g.beginPath();g.arc(c,c,m*k,0,2*Math.PI);g.stroke();}
  const d=(s.lidar&&s.lidar.d)||[],N=d.length,st=360/Math.max(N,1);
  const P=d.map((cm,i)=>{if(!cm||cm>ENV_MAX_CM)return null;const a=(i*st+st/2)*Math.PI/180;
    return [c+cm*k*Math.sin(a),c-cm*k*Math.cos(a),cm];});
  const near=(p,q)=>p&&q&&Math.abs(p[2]-q[2])<Math.max(30,0.25*Math.min(p[2],q[2]));
  g.strokeStyle='#4ea1ff';g.fillStyle='#4ea1ff';g.lineWidth=3;g.lineCap='round';
  for(let i=0;i<N;i++){const p=P[i];if(!p)continue;const q=P[(i+1)%N],o=P[(i+N-1)%N];
    if(near(p,q)){g.beginPath();g.moveTo(p[0],p[1]);g.lineTo(q[0],q[1]);g.stroke();}
    else if(!near(o,p)){g.beginPath();g.arc(p[0],p[1],2.5,0,2*Math.PI);g.fill();}}
  g.fillStyle='#e7edf5';g.fillRect(c-9,c-12,18,24);  // box
  g.fillStyle='#ffb020';g.beginPath();g.moveTo(c,c-20);g.lineTo(c-6,c-12);g.lineTo(c+6,c-12);g.fill();  // front
}

// ---- sensor cards
function renderSensors(s){
  if(!s)return;
  drawEnv(s);
  const L=s.lidar||{};
  $('lidarInfo').textContent=L.ok?('LiDAR '+L.pps+' packets/s, checksum errors '+L.crc+' %'):'LiDAR: no data';
  const us=s.us||[],names=['Front','Left','Right','Back','Front angled'];
  $('usTable').innerHTML=names.map((n,i)=>`<tr><td class='k'>Ultrasonic ${n}</td><td>${
    (us[i]===undefined||us[i]>=s.usMax)?'-':us[i]+' cm'}</td></tr>`).join('');
  const I=s.imu||{},G=s.gps||{};
  // ISO 8855: roll + = right side down, pitch + = front down, yaw + = turning left (0 = north)
  let t=I.ok?`<tr><td class='k'>Roll / pitch / yaw (ISO 8855)</td><td>${I.roll}&deg; / ${I.pitch}&deg; / ${I.yaw}&deg;</td></tr>`
    +`<tr><td class='k'>IMU calibration sys/gyr/acc/mag</td><td>${(I.cal||[]).join(' / ')} (3 = good)</td></tr>`
    +`<tr><td class='k'>IMU mode</td><td>${I.compass?(I.calSaved?'compass on, saved calibration loaded (levels read 0 until the rover moved)':'compass on, no saved calibration')
      :'compass off (gyro mode): only Gyro must be 3, keep the rover still ~3 s after power-on'}</td></tr>`
    :`<tr><td class='k'>IMU</td><td>not found</td></tr>`;
  t+=G.chars?`<tr><td class='k'>GPS</td><td>${G.fix?'fix':'no fix'}, ${G.sats} satellites, HDOP ${G.hdop}</td></tr>`
    :`<tr><td class='k'>GPS</td><td>no data from the module</td></tr>`;
  if(G.fix)t+=`<tr><td class='k'>Latitude / longitude</td><td>${G.lat}, ${G.lon}</td></tr>`;
  $('navTable').innerHTML=t;
}

// ---- run card
function renderRun(R,I){
  if(!R)return;
  const b=$('runBadge');
  b.className='badge '+(R.state==='RUNNING'?(R.paused?'b-pause':'b-run'):(R.state==='STOPPED'?'b-stop':'b-idle'));
  b.textContent=(R.state==='RUNNING'&&!R.paused)?String.fromCharCode(9654)+' '+R.nav:R.text;
  const sb=$('safBtn');sb.textContent='Safety stops: '+(R.safety?'ON':'OFF');sb.style.background=R.safety?'':'#d97706';
  const w=R.wall?`${R.wall} cm / ${R.wallDeg}&deg;${R.wallSteer?' (steering)':''}`:'-';
  const rows=[['Poles passed',R.poles+' / '+R.polesTarget],['Run time',R.elapsedS+' s'],
    ['Turn rate',(I&&I.ok)?I.rate+' &deg;/s':'-'],['Heading ref / cross-track',R.headingRef+'&deg; / '+R.crossTrack+' cm'],
    ['Lane check ahead',R.front?R.front+' cm':'clear'],['Left wall',w]];
  $('runTable').innerHTML=rows.map(r=>`<tr><td class='k'>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
}

// ---- poll the box 4 times per second; one request at a time (a dead link often hangs instead of
//      failing, so requests would pile up), banner when the box has not answered for 1 s
let lastOk=Date.now(),busy=false;
async function poll(){
  if(busy)return;
  busy=true;
  const ctl=new AbortController(),t=setTimeout(()=>ctl.abort(),1000);
  try{
    const j=await (await fetch('/values',{cache:'no-store',signal:ctl.signal})).json();
    lastOk=Date.now();
    $('conn').textContent='Online';
    $('state').textContent=j.state||'-';
    renderSensors(j.sensors);
    renderRun(j.run,j.sensors&&j.sensors.imu);
    $('msg').textContent=j.msg||'';
    if(dev){
      $('valTable').innerHTML=(j.values||[]).map(v=>`<tr><td class='k'>${esc(v.name)}</td><td>${esc(v.value)}</td></tr>`).join('');
      const c=j.cycle||{};$('cycle').textContent=`${c.min} / ${c.avg} / ${c.max} ms`;
    }
  }catch(e){$('conn').textContent='Offline';}
  finally{clearTimeout(t);busy=false;}
}
setInterval(poll,250);poll();
setInterval(()=>{$('lost').classList.toggle('hidden',Date.now()-lastOk<=1000);},250);
</script></body></html>)rawpage";

// Firmware update page. %FW% = running version.
static const char kUpdatePage[] PROGMEM = R"rawpage(<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'><title>Firmware update</title>
<style>body{font-family:system-ui,sans-serif;background:#0b0f17;color:#e7edf5;padding:16px}
input,button{font-size:16px;margin:8px 0;width:100%}button{padding:14px;border:0;border-radius:12px;background:#4ea1ff;color:#04101f;font-weight:700}
p{color:#9aa4b2}a{color:#4ea1ff}</style></head><body>
<h2>Firmware update over Wi-Fi</h2><p>Running: %FW%</p>
<p>Arduino IDE: Sketch &rarr; Export Compiled Binary, then choose <b>box.ino.bin</b> (not .merged.bin /
.bootloader.bin). Stop the run first. The box restarts when done (also after a failed upload).</p>
<form method='POST' action='/update' enctype='multipart/form-data'>
<input type='file' name='firmware' accept='.bin'><button type='submit'>Upload &amp; restart</button></form>
<p><a href='/'>Back</a></p></body></html>)rawpage";

// Appends s to out as JSON string content (quotes and backslashes escaped, control chars dropped).
static void jsonEscape(String& output, const char* text) {
  for (; *text; ++text) {
    const char character = *text;
    if (character == '"' || character == '\\') { output += '\\'; output += character; }
    else if ((uint8_t)character >= 0x20) output += character;
  }
}

// ---------------------------------------------------------------------------------------------

// Takes the mutex; creates it on first use (always in setup(), before the server task runs).
void WebHMI::_lock() {
  if (!_dataMutex) _dataMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(_dataMutex, portMAX_DELAY);
}

void WebHMI::begin(const char* ssid, const char* password, IPAddress ip, uint8_t channel) {
  _lock();  // make sure the mutex exists before the server starts
  xSemaphoreGive(_dataMutex);
  if (!_commandQueue) _commandQueue = xQueueCreate(8, sizeof(uint8_t));

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                                  // no Wi-Fi power save on the box
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(ssid, password, channel)) {
    Serial.printf("[WebHMI] AP %s, channel %d, IP %s\n", ssid, WiFi.channel(),
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WebHMI] WiFi.softAP failed");
  }

  _server = new AsyncWebServer(80);
  _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
    request->send(200, "text/html; charset=utf-8", _buildPage());
  });
  _server->on("/values", HTTP_GET, [this](AsyncWebServerRequest* request) {
    _lastPollMs = millis();  // the page is alive: feeds the link watchdog of the run
    request->send(200, "application/json", _buildJson());
  });
  _server->on("/drive", HTTP_GET, [this](AsyncWebServerRequest* request) {
    if (!request->hasParam("l") || !request->hasParam("r") || !request->hasParam("id") || !request->hasParam("n")) {
      request->send(400, "application/json", "{\"ok\":false}");
      return;
    }
    const float left = constrain(request->getParam("l")->value().toFloat(), -_driveMaxSpeed, _driveMaxSpeed);
    const float right = constrain(request->getParam("r")->value().toFloat(), -_driveMaxSpeed, _driveMaxSpeed);
    const uint32_t pageId = strtoul(request->getParam("id")->value().c_str(), nullptr, 10);
    const uint32_t sequence = strtoul(request->getParam("n")->value().c_str(), nullptr, 10);
    bool isFresh;
    portENTER_CRITICAL(&_driveMux);
    // a new page (id) starts over; within a page only a higher number counts
    isFresh = (pageId != _drivePageId) || (sequence > _driveSequence);
    if (isFresh) {
      _drivePageId = pageId;
      _driveSequence = sequence;
      _driveLeft = left;
      _driveRight = right;
      _driveReceivedMs = millis();
    }
    portEXIT_CRITICAL(&_driveMux);
    request->send(200, "application/json", isFresh ? "{\"ok\":true}" : "{\"ok\":false,\"stale\":true}");
  });
  if (_logger) {
    _server->on("/log", HTTP_GET, [this](AsyncWebServerRequest* request) { _handleLog(request); });
  }
  _server->on("/run/start", HTTP_GET, [this](AsyncWebServerRequest* request) { _queueCommand(request, CMD_RUN_START); });
  _server->on("/run/stop", HTTP_GET, [this](AsyncWebServerRequest* request) { _queueCommand(request, CMD_RUN_STOP); });
  _server->on("/run/safety", HTTP_GET, [this](AsyncWebServerRequest* request) { _queueCommand(request, CMD_SAFETY_TOGGLE); });
  _server->on("/imu/save", HTTP_GET, [this](AsyncWebServerRequest* request) { _queueCommand(request, CMD_IMU_SAVE); });
  if (_otaEnabled) {
    _server->on("/update", HTTP_GET, [this](AsyncWebServerRequest* request) {
      String updatePage = FPSTR(kUpdatePage);
      updatePage.replace("%FW%", _firmwareVersion);
      request->send(200, "text/html; charset=utf-8", updatePage);
    });
    _server->on("/update", HTTP_POST,
      [this](AsyncWebServerRequest* request) {  // after the upload
        if (request != _otaRequest) {
          request->send(409, "text/plain", _otaActive ? "Another update is running." : "Refused: stop the run first.");
          return;
        }
        const bool isUploadOk = Update.isFinished() && !Update.hasError();
        request->send(200, "text/plain", isUploadOk ? String("Update OK - restarting. Reconnect to the Wi-Fi in ~10 s.")
                                        : String("Update FAILED: ") + Update.errorString() + " - restarting with the old firmware.");
        _restartAtMs = (millis() + 800) | 1;  // let the answer go out; restart after any attempt
      },
      [this](AsyncWebServerRequest* request, const String& fileName, size_t offset, uint8_t* data, size_t length, bool isFinal) {
        _handleUpload(request, fileName, offset, data, length, isFinal);
      });
  }
  _server->begin();
  _cycleLastUs = micros();
  _cycleWindowStartMs = millis();
}

void WebHMI::setTitle(const String& title) { _title = title; }

void WebHMI::setRunJson(const String& json) {
  _lock();
  _runJson = json;
  xSemaphoreGive(_dataMutex);
}

void WebHMI::setMessage(const String& text) {
  _lock();
  _message = text;
  xSemaphoreGive(_dataMutex);
}

uint32_t WebHMI::pageAgeMs() const {
  const uint32_t lastPollMs = _lastPollMs;
  return lastPollMs ? millis() - lastPollMs : UINT32_MAX;  // millis() read after t: never negative
}

bool WebHMI::takeCommand(Command& command) {
  uint8_t commandByte;
  if (!_commandQueue || xQueueReceive(_commandQueue, &commandByte, 0) != pdTRUE) return false;
  command = (Command)commandByte;
  return true;
}

void WebHMI::_queueCommand(AsyncWebServerRequest* request, Command command) {
  const uint8_t commandByte = command;
  const bool isQueued = _commandQueue && xQueueSend(_commandQueue, &commandByte, 0) == pdTRUE;
  request->send(isQueued ? 200 : 503, "text/plain", isQueued ? "queued" : "busy");
}

// Upload chunks (server task). One upload at a time, refused while a run is active
// (setUpdateAllowed). The ultrasonic task is suspended and the rover stopped by loop() as soon as
// updateActive() is true.
void WebHMI::_handleUpload(AsyncWebServerRequest* request, const String& fileName, size_t offset, uint8_t* data,
                           size_t length, bool isFinal) {
  if (offset == 0) {
    if (_otaRequest || !_otaAllowed) {
      Serial.println("[ota] upload refused");
      return;
    }
    _otaRequest = request;
    _otaActive = true;
    _otaLastDataMs = millis();
    Serial.printf("[ota] receiving %s, free heap %u\n", fileName.c_str(), ESP.getFreeHeap());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  }
  if (request != _otaRequest) return;
  _otaLastDataMs = millis();
  if (length && Update.write(data, length) != length) Update.printError(Serial);
  if (isFinal) {
    if (Update.end(true)) Serial.printf("[ota] done, %u bytes\n", (unsigned)(offset + length));
    else Update.printError(Serial);
  }
}

void WebHMI::enableDrivePad(float maxSpeed) {
  _isDrivePadEnabled = true;
  _driveMaxSpeed = maxSpeed;
}

void WebHMI::setStateText(const String& text) {
  _lock();
  _state = text;
  xSemaphoreGive(_dataMutex);
}

void WebHMI::setSensorJson(const String& json) {
  _lock();
  _sensorJson = json;
  xSemaphoreGive(_dataMutex);
}

void WebHMI::showValue(const char* name, double value) { _setValue(name, String(value, 3)); }
void WebHMI::showValue(const char* name, const String& value) { _setValue(name, value); }

// Updates the value with this name or takes the first free slot (ignored if all are used).
void WebHMI::_setValue(const char* name, const String& text) {
  _lock();
  Value* slot = nullptr;
  for (Value& value : _values) {
    if (value.inUse && strncmp(value.name, name, sizeof(value.name)) == 0) { slot = &value; break; }
    if (!value.inUse && !slot) slot = &value;
  }
  if (slot) {
    if (!slot->inUse) {
      strlcpy(slot->name, name, sizeof(slot->name));
      slot->inUse = true;
    }
    strlcpy(slot->text, text.c_str(), sizeof(slot->text));
  }
  xSemaphoreGive(_dataMutex);
}

void WebHMI::getDrive(float& left, float& right, uint32_t& commandAgeMs) const {
  portENTER_CRITICAL(&_driveMux);
  left = _driveLeft;
  right = _driveRight;
  const uint32_t receivedMs = _driveReceivedMs;
  portEXIT_CRITICAL(&_driveMux);
  commandAgeMs = receivedMs ? millis() - receivedMs : UINT32_MAX;  // millis() read after ms: never negative
}

// Loop time over 1 s windows: min / average / max time between two tick() calls.
void WebHMI::tick() {
  const uint32_t restartAtMs = _restartAtMs;
  if (restartAtMs && (int32_t)(millis() - restartAtMs) >= 0) {
    Serial.println("[ota] restarting");
    delay(100);
    ESP.restart();
  }
  if (_otaActive && !restartAtMs && millis() - _otaLastDataMs > 15000) {  // upload stalled (connection lost)
    Update.abort();
    Serial.println("[ota] upload stalled, restarting with the old firmware");
    delay(100);
    ESP.restart();
  }

  const uint32_t nowUs = micros();
  const uint32_t cycleUs = nowUs - _cycleLastUs;
  _cycleLastUs = nowUs;
  if (_cycleCount == 0 || cycleUs < _cycleMinUs) _cycleMinUs = cycleUs;
  if (cycleUs > _cycleMaxUs) _cycleMaxUs = cycleUs;
  _cycleSumUs += cycleUs;
  _cycleCount++;

  const uint32_t nowMs = millis();
  if (nowMs - _cycleWindowStartMs < 1000) return;
  _cycleWindowStartMs = nowMs;
  _lock();
  _cycleMinMs = _cycleMinUs / 1000.0f;
  _cycleMaxMs = _cycleMaxUs / 1000.0f;
  _cycleAvgMs = _cycleSumUs / 1000.0f / _cycleCount;
  xSemaphoreGive(_dataMutex);
  _cycleCount = 0;
  _cycleSumUs = 0;
  _cycleMaxUs = 0;
}

String WebHMI::_buildPage() const {
  String page = FPSTR(kPage);
  page.replace("%TITLE%", _title);
  page.replace("%MAXSPD%", _isDrivePadEnabled ? String(_driveMaxSpeed, 2) : String("0"));
  String firmwareLine = String("firmware ") + _firmwareVersion;
  if (_otaEnabled) firmwareLine += " &middot; <a href='/update'>update over Wi-Fi</a>";
  page.replace("%FW%", firmwareLine);
  return page;
}

// {"state":"..","values":[{"name":..,"value":..}],"cycle":{min,avg,max},"sensors":{..}|null,
//  "run":{..}|null,"msg":"..","fw":".."}
String WebHMI::_buildJson() const {
  String json;
  json.reserve(1700);
  xSemaphoreTake(_dataMutex, portMAX_DELAY);
  json += "{\"state\":\"";
  jsonEscape(json, _state.c_str());
  json += "\",\"values\":[";
  bool isFirst = true;
  for (const Value& value : _values) {
    if (!value.inUse) continue;
    if (!isFirst) json += ',';
    isFirst = false;
    json += "{\"name\":\"";
    jsonEscape(json, value.name);
    json += "\",\"value\":\"";
    jsonEscape(json, value.text);
    json += "\"}";
  }
  json += "],\"cycle\":{\"min\":";
  json += String(_cycleMinMs, 2);
  json += ",\"avg\":";
  json += String(_cycleAvgMs, 2);
  json += ",\"max\":";
  json += String(_cycleMaxMs, 2);
  json += "},\"sensors\":";
  json += _sensorJson.length() ? _sensorJson : String("null");
  json += ",\"run\":";
  json += _runJson.length() ? _runJson : String("null");
  json += ",\"msg\":\"";
  jsonEscape(json, _message.c_str());
  json += "\",\"fw\":\"";
  jsonEscape(json, _firmwareVersion);
  json += "\"}";
  xSemaphoreGive(_dataMutex);
  return json;
}

// ---------------------------------------------------------------------------------------------
// /log routes (docs/logging.md). Runs in the server task; every file access goes through the
// Logger's own mutex (files(), readChunk(), clear()), mark() only takes its spinlock.
// ---------------------------------------------------------------------------------------------

void WebHMI::_handleLog(AsyncWebServerRequest* request) {
  // during an autonomous run the log is in RAM and the flash stays untouched (a flash read also
  // freezes both cores): downloads and clear wait until the run is written
  const bool isFlashBlocked = _logger->isRunActive() || _logger->isRunWritePending();
  if (isFlashBlocked && (request->hasParam("get") || request->hasParam("clear"))) {
    request->send(409, "text/plain", "Autonomous run: the log is written after the run. Try again then.");
    return;
  }
  if (request->hasParam("get")) {
    const String fileName = request->getParam("get")->value();
    if (!Logger::validName(fileName.c_str())) {
      request->send(400, "text/plain", "bad file name");
      return;
    }
    // size from the listing; the current file may grow meanwhile, the download ends at this size
    Logger::FileInfo fileList[Logger::kMaxFiles];
    const uint8_t fileCount = _logger->files(fileList, Logger::kMaxFiles);
    size_t fileSize = 0;
    bool isFound = false;
    for (uint8_t i = 0; i < fileCount && !isFound; i++) {
      if (fileName == fileList[i].name) { fileSize = fileList[i].size; isFound = true; }
    }
    if (!isFound) {
      request->send(404, "text/plain", "no such log file");
      return;
    }
    Logger* logger = _logger;
    AsyncWebServerResponse* response = request->beginResponse(
        "text/csv", fileSize, [logger, fileName, fileSize](uint8_t* buffer, size_t maxLength, size_t offset) -> size_t {
          if (offset >= fileSize) return 0;
          if (maxLength > fileSize - offset) maxLength = fileSize - offset;
          const size_t bytesRead = logger->readChunk(fileName.c_str(), buffer, maxLength, offset);
          return bytesRead == (size_t)-1 ? RESPONSE_TRY_AGAIN : bytesRead;  // flash busy: the server retries
        });
    response->addHeader("Content-Disposition", String("attachment; filename=\"") + fileName + "\"");
    request->send(response);
    return;
  }
  if (request->hasParam("clear")) {
    const bool isCleared = _logger->clear();
    request->send(isCleared ? 200 : 503, "text/plain", isCleared ? "cleared" : "clear failed (flash busy or no filesystem)");
    return;
  }
  if (request->hasParam("mark")) {
    _logger->mark(request->getParam("mark")->value().c_str());
    request->send(200, "text/plain", "marked");
    return;
  }
  request->send(200, "text/html; charset=utf-8", _buildLogPage());
}

// Log page: status, files (newest last, each a download link), mark field, clear button.
String WebHMI::_buildLogPage() {
  Logger::FileInfo fileList[Logger::kMaxFiles];
  const bool isFlashBlocked = _logger->isRunActive() || _logger->isRunWritePending();
  const uint8_t fileCount = isFlashBlocked ? 0 : _logger->files(fileList, Logger::kMaxFiles);
  String html;
  html.reserve(1500 + fileCount * 120);
  html += F("<!doctype html><html><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width, initial-scale=1'><title>");
  html += _title;
  html += F(" log</title><style>body{margin:0;padding:16px;background:#0b0f17;color:#e7edf5;"
           "font-family:system-ui,sans-serif}a{color:#4ea1ff}td{padding:6px 10px}"
           "input,button{font-size:15px;padding:8px}</style></head><body><h3>");
  html += _title;
  html += F(" timeline log</h3><p>");
  html += _logger->status();
  html += F("</p><table>");
  for (uint8_t i = 0; i < fileCount; i++) {
    html += F("<tr><td><a href='/log?get=");
    html += fileList[i].name;  // validName(): letters, digits, '_' and '.' only
    html += F("'>");
    html += fileList[i].name;
    html += F("</a></td><td>");
    html += String((uint32_t)((fileList[i].size + 1023) / 1024));
    html += F(" kB</td></tr>");
  }
  if (!fileCount) html += isFlashBlocked ? F("<tr><td>files are listed after the run</td></tr>")
                                         : F("<tr><td>no log files</td></tr>");
  html += F("</table><p><input id='m' maxlength='23' placeholder='note, e.g. cp1'> "
           "<button onclick=\"fetch('/log?mark='+encodeURIComponent(document.getElementById('m').value))"
           ".then(r=>r.text()).then(t=>{document.getElementById('m').value='';alert(t)})\">Mark</button></p>"
           "<p><button onclick=\"if(confirm('Delete all logs on the box?'))fetch('/log?clear=1')"
           ".then(r=>r.text()).then(t=>{alert(t);location.reload()})\">Clear all logs</button></p>"
           "<p><a href='/'>back</a></p></body></html>");
  return html;
}
