String boolJson(bool value) { return value ? "true" : "false"; }
String numberOrNull(float value, uint8_t digits = 3) { return isfinite(value) ? String(value, digits) : String("null"); }

void apiStatus() {
  const uint64_t t = nowUs();
  const auto& g = gnssRx.latest();
  char offsetText[32]{};
  snprintf(offsetText, sizeof(offsetText), "%lld", static_cast<long long>(controlMinusCommunicationUs));
  String json;
  json.reserve(3500);
  json += "{\"firmware\":\"" + String(kFirmwareVersion) + "\",";
  json += "\"sd_ready\":" + boolJson(sdReady) + ",\"logging\":" + boolJson(logging) + ",\"log_file\":\"" + activeLogName + "\",\"log_records\":" + String(logRecords) + ",\"log_drops\":" + String(logDrops) + ",";
  json += "\"link\":{\"age_ms\":" + String(lastControlRxUs ? (t - lastControlRxUs) / 1000ULL : UINT32_MAX) + ",\"rx\":" + String(rxFrames) + ",\"tx\":" + String(txFrames) + ",\"drops\":" + String(txDrops) + ",\"gaps\":" + String(rxSequenceGaps) + ",\"crc\":" + String(controlDecoder.crcErrors) + ",\"cobs\":" + String(controlDecoder.cobsErrors) + "},";
  json += "\"sync\":{\"offset_us\":" + String(offsetText) + ",\"rtt_us\":" + String(syncRttUs) + ",\"uncertainty_us\":" + String(syncUncertaintyUs) + "},";
  json += "\"gnss\":{\"receiving\":" + boolJson(gnssRx.receiving(t)) + ",\"fix\":" + boolJson(g.flags & gnss::FixValid) + ",\"lat\":" + String(g.latitude, 7) + ",\"lon\":" + String(g.longitude, 7) + ",\"speed\":" + String(g.speedMps, 3) + ",\"course\":" + String(g.courseDeg, 2) + ",\"sats\":" + String(g.satellites) + "},";
  json += "\"health\":{\"valid\":" + boolJson(cache.healthValid) + ",\"flags\":" + String(cache.health.flags) + ",\"state\":" + String(cache.health.safetyState) + ",\"mode\":" + String(cache.health.controlMode) + ",\"reason\":" + String(cache.health.stopReason) + "},";
  json += "\"attitude\":{\"valid\":" + boolJson(cache.rotationValid) + ",\"accuracy\":" + String(cache.rotation.accuracy) + ",\"roll\":" + numberOrNull(cache.rotation.v[4]) + ",\"pitch\":" + numberOrNull(cache.rotation.v[5]) + ",\"yaw\":" + numberOrNull(cache.rotation.v[6]) + "},";
  json += "\"control\":{\"valid\":" + boolJson(cache.controlValid) + ",\"wp\":" + String(cache.control.activeWaypoint) + ",\"wp_count\":" + String(cache.control.waypointCount) + ",\"wp_distance\":" + numberOrNull(cache.control.waypointDistanceM) + ",\"bearing_deg\":" + numberOrNull(cache.control.targetBearingRad * 180.0f / PI, 2) + ",\"height\":" + numberOrNull(cache.control.heightM) + ",\"left\":" + numberOrNull(cache.control.leftCommand) + ",\"right\":" + numberOrNull(cache.control.rightCommand) + ",\"rear\":" + numberOrNull(cache.control.rearCommand) + ",\"propulsion\":" + numberOrNull(cache.control.propulsionCommand) + "},";
  json += "\"actuator\":{\"valid\":" + boolJson(cache.actuatorValid) + ",\"relay\":" + boolJson(cache.actuator.motorRelayEnabled) + ",\"mask\":" + String(cache.actuator.enabledMask) + ",\"left_us\":" + String(cache.actuator.leftPulseUs) + ",\"right_us\":" + String(cache.actuator.rightPulseUs) + ",\"rear_us\":" + String(cache.actuator.rearPulseUs) + ",\"target_duty\":" + numberOrNull(cache.actuator.targetDuty) + ",\"applied_duty\":" + numberOrNull(cache.actuator.appliedDuty) + "},";
  json += "\"vesc\":{\"valid\":" + boolJson(cache.vesc.valid) + ",\"fault\":" + String(cache.vesc.fault) + ",\"voltage\":" + numberOrNull(cache.vesc.inputVoltageV) + ",\"motor_current\":" + numberOrNull(cache.vesc.motorCurrentA) + ",\"input_current\":" + numberOrNull(cache.vesc.inputCurrentA) + ",\"duty\":" + numberOrNull(cache.vesc.duty) + ",\"erpm\":" + numberOrNull(cache.vesc.erpm, 1) + ",\"mos_temp\":" + numberOrNull(cache.vesc.mosTempC) + "},";
  json += "\"ina\":{\"valid\":" + boolJson(cache.ina.valid) + ",\"voltage\":" + numberOrNull(cache.ina.busVoltageV) + ",\"current\":" + numberOrNull(cache.ina.currentA) + ",\"power\":" + numberOrNull(cache.ina.powerW) + "},";
  json += "\"ack\":{\"valid\":" + boolJson(cache.ackValid) + ",\"type\":" + String(cache.ack.commandType) + ",\"disposition\":" + String(cache.ack.disposition) + ",\"reason\":" + String(cache.ack.reason) + ",\"sequence\":" + String(cache.ack.commandSequence) + "}}";
  web.send(200, "application/json", json);
}

void apiAccepted() { web.send(202, "application/json", "{\"accepted\":true}"); }
void apiError(int code, const char* error) { web.send(code, "application/json", String("{\"error\":\"") + error + "\"}"); }

const char page[] PROGMEM = R"HTML(
<!doctype html><html lang="ja"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{font:15px system-ui;background:#0d141c;color:#edf3fa;margin:12px;max-width:900px}.card{background:#192633;border-radius:10px;padding:12px;margin:9px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:8px}button,select,input,textarea{font:inherit;padding:8px;margin:3px;border-radius:6px;border:1px solid #526578}button{cursor:pointer}label{display:block}.danger{background:#a72b32;color:white}.stop{background:#d68b21;color:#111}.run{background:#3ea86a;color:#07120b}pre{white-space:pre-wrap;overflow-wrap:anywhere}textarea{width:95%;min-height:90px;background:#0b1118;color:#edf3fa}</style>
<h2>2台XIAO 水上翼船</h2>
<div class="card"><pre id="status">接続待ち</pre></div>
<div class="card"><h3>モード・手動指令</h3><div class="grid">
<label>モード<select id="mode"><option value="manual">Manual</option><option value="attitude">Attitude Assist</option><option value="heading">Heading Hold</option><option value="auto">Auto Waypoint</option></select></label>
<label><input type="checkbox" id="leftOn" checked> 左前翼 CH0</label><label><input type="checkbox" id="rightOn"> 右前翼 CH1</label><label><input type="checkbox" id="rearOn"> 後部ヨー CH2</label><label><input type="checkbox" id="propOn"> 推進</label>
<label>左 <input type="range" id="left" min="-1" max="1" step=".01" value="0"><span id="leftV">0</span></label>
<label>右 <input type="range" id="right" min="-1" max="1" step=".01" value="0"><span id="rightV">0</span></label>
<label>後部 <input type="range" id="rear" min="-1" max="1" step=".01" value="0"><span id="rearV">0</span></label>
<label>推進Duty <input type="range" id="prop" min="0" max="0.6" step=".01" value="0"><span id="propV">0</span></label>
<label>目標Yaw[deg] <input id="heading" type="number" value="0" step="1"></label></div>
<button onclick="applyConfig()">設定送信</button><button class="run" onclick="runBoat()">ARMして開始</button><button class="stop" onclick="post('/api/control/stop')">停止</button><button onclick="post('/api/control/disarm')">DISARM</button><button class="danger" onclick="post('/api/control/estop')">緊急停止</button><button onclick="post('/api/control/clear-estop')">E-STOP解除</button></div>
<div class="card"><h3>ウェイポイント</h3><p>1行につき latitude,longitude。変更はDISARMED中だけです。</p><textarea id="points" placeholder="36.0000000,136.0000000"></textarea><label>到達半径[m] <input id="reach" type="number" value="1.5" min="0.5" step="0.1"></label><button onclick="sendWaypoints()">経路送信</button></div>
<div class="card"><h3>ログ</h3><button onclick="post('/api/log/start')">記録開始</button><button onclick="post('/api/log/stop')">記録停止</button></div>
<script>
const ids=['left','right','rear','prop'];ids.forEach(x=>document.getElementById(x).oninput=()=>document.getElementById(x+'V').textContent=document.getElementById(x).value);
const el=id=>document.getElementById(id);
function args(){let mask=(el('leftOn').checked?1:0)|(el('rightOn').checked?2:0)|(el('rearOn').checked?4:0)|(el('propOn').checked?8:0);return {mode:el('mode').value,mask,left:el('left').value,right:el('right').value,rear:el('rear').value,propulsion:el('prop').value,heading:el('heading').value}}
async function post(path,data={}){let body=new URLSearchParams(data);let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});let t=await r.text();if(!r.ok)alert(t);return r.ok}
async function applyConfig(){let a=args();await post('/api/control/mode',{mode:a.mode});await post('/api/control/manual',a);if(a.mode==='heading')await post('/api/control/heading',{degrees:a.heading})}
async function runBoat(){await post('/api/control/run',args())}
async function sendWaypoints(){await post('/api/waypoints',{points:el('points').value,reach:el('reach').value})}
const stateNames=['BOOT','DISARMED','ARMED_IDLE','RUNNING','E_STOP','FAULT'];const modeNames=['MANUAL','ATTITUDE','HEADING','AUTO'];
async function update(){try{let j=await(await fetch('/api/status',{cache:'no-store'})).json();let h=j.health,a=j.actuator,v=j.vesc,c=j.control,g=j.gnss,k=j.ack;el('status').textContent=`状態 ${stateNames[h.state]||h.state} / ${modeNames[h.mode]||h.mode} / reason ${h.reason}\nリレー ${a.relay?'HIGH':'LOW'}  Duty target/applied ${a.target_duty}/${a.applied_duty}\nServo L/R/Y ${a.left_us}/${a.right_us}/${a.rear_us} us mask=${a.mask}\n姿勢 Roll/Pitch/Yaw ${j.attitude.roll}/${j.attitude.pitch}/${j.attitude.yaw} deg accuracy=${j.attitude.accuracy}\nGNSS ${g.fix?'FIX':'NO FIX'} ${g.lat}, ${g.lon} speed=${g.speed} sats=${g.sats}\nWP ${c.wp}/${c.wp_count} distance=${c.wp_distance}m bearing=${c.bearing_deg}deg height=${c.height}m\nVESC valid=${v.valid} fault=${v.fault} V=${v.voltage} I=${v.motor_current}A ERPM=${v.erpm} temp=${v.mos_temp}\nINA valid=${j.ina.valid} V=${j.ina.voltage} I=${j.ina.current} P=${j.ina.power}\nUART age=${j.link.age_ms}ms RX/TX=${j.link.rx}/${j.link.tx} gaps=${j.link.gaps} CRC=${j.link.crc}\n時刻同期 offset=${j.sync.offset_us}us RTT=${j.sync.rtt_us}us\nSD=${j.sd_ready} log=${j.logging} ${j.log_file} records=${j.log_records} drops=${j.log_drops}\nACK type=${k.type} result=${k.disposition} reason=${k.reason} seq=${k.sequence}`;}catch(e){el('status').textContent='更新エラー '+e}setTimeout(update,100)}update();
</script></html>)HTML";

void beginWeb() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  web.on("/", HTTP_GET, [] { web.send(200, "text/html; charset=utf-8", page); });
  web.on("/api/status", HTTP_GET, apiStatus);
  web.on("/api/log/start", HTTP_POST, [] { if (!sdReady) { apiError(503, "sd_unavailable"); return; } logCommand = 1; apiAccepted(); });
  web.on("/api/log/stop", HTTP_POST, [] { logCommand = 2; apiAccepted(); });
  web.on("/api/control/mode", HTTP_POST, [] { sendMode(parseMode(web.arg("mode"))); apiAccepted(); });
  web.on("/api/control/manual", HTTP_POST, [] { updateManualFromWeb(); if (!manualConfig.configured) { apiError(400, "no_output_selected"); return; } sendManual(); apiAccepted(); });
  web.on("/api/control/heading", HTTP_POST, [] { sendHeading(web.arg("degrees").toFloat()); apiAccepted(); });
  web.on("/api/control/arm", HTTP_POST, [] { enqueueControl(boat::Type::Arm); apiAccepted(); });
  web.on("/api/control/disarm", HTTP_POST, [] { enqueueControl(boat::Type::Disarm); apiAccepted(); });
  web.on("/api/control/start", HTTP_POST, [] { enqueueControl(boat::Type::Start); apiAccepted(); });
  web.on("/api/control/stop", HTTP_POST, [] { enqueueControl(boat::Type::Stop); apiAccepted(); });
  web.on("/api/control/estop", HTTP_POST, [] { enqueueControl(boat::Type::Estop); apiAccepted(); });
  web.on("/api/control/clear-estop", HTTP_POST, [] { enqueueControl(boat::Type::ClearEstop); apiAccepted(); });
  web.on("/api/control/run", HTTP_POST, [] {
    updateManualFromWeb();
    const boat::ControlMode mode = parseMode(web.arg("mode"));
    if (mode == boat::ControlMode::Manual && !manualConfig.configured) { apiError(400, "no_output_selected"); return; }
    if (sdReady && !logging) logCommand = 1;
    sendMode(mode);
    if (manualConfig.configured) sendManual();
    if (mode == boat::ControlMode::HeadingHold) sendHeading(web.arg("heading").toFloat());
    enqueueControl(boat::Type::Arm);
    enqueueControl(boat::Type::Start);
    apiAccepted();
  });
  web.on("/api/waypoints", HTTP_POST, [] {
    boat::WaypointSetPayload payload{};
    payload.requestId = nextRequestId();
    payload.revision = payload.requestId;
    payload.action = 1;
    payload.reachRadiusM = clampf(web.arg("reach").toFloat(), 0.5f, 20.0f);
    if (!parseWaypoints(web.arg("points"), payload)) { apiError(400, "invalid_waypoints"); return; }
    payload.canonicalCrc = boat::canonicalCrc(&payload, offsetof(boat::WaypointSetPayload, canonicalCrc));
    enqueueControl(boat::Type::WaypointSet, &payload, sizeof(payload));
    apiAccepted();
  });
  web.begin();
}

