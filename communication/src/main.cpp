#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <esp_timer.h>
#include "app_config.h"
#include "gnss_receiver.h"
#include "boat_protocol.h"

using namespace app_config;
namespace {
HardwareSerial controlUart(1), gnssUart(2);
WebServer web(kHttpPort);
gnss::Receiver gnssRx;
boat::Decoder decoder;
QueueHandle_t controlTxQueue = nullptr, logQueue = nullptr;
portMUX_TYPE cacheMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t bootId = 0, txSequence = 0, navSequence = 0, lastNavMs = 0, lastHeartbeatMs = 0, lastDiagMs = 0;
uint64_t lastControlUs = 0;
uint32_t rxFrames = 0, txFrames = 0, linkDrops = 0, logDrops = 0, logRecords = 0;
bool sdReady = false, logging = false;
String activeLogName;
volatile uint8_t logCommand = 0;  // 1=start, 2=stop

struct TxItem { boat::Type type; uint16_t length; uint8_t payload[boat::kMaxPayload]; };
struct LogItem { boat::Header header; uint16_t length; uint8_t payload[boat::kMaxPayload]; };
struct RotationCache { boat::BnoPayload payload{}; uint64_t receivedUs = 0; bool valid = false; } rotation;

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }

bool enqueueControl(boat::Type type, const void* payload = nullptr, size_t length = 0) {
  if (!controlTxQueue || length > boat::kMaxPayload) return false;
  TxItem item{}; item.type = type; item.length = static_cast<uint16_t>(length);
  if (length) memcpy(item.payload, payload, length);
  if (xQueueSend(controlTxQueue, &item, 0) != pdPASS) { ++linkDrops; return false; }
  return true;
}

void enqueueLog(const boat::Header& header, const uint8_t* payload) {
  if (!logging || !logQueue) return;
  LogItem item{}; item.header = header; item.length = header.length;
  if (item.length) memcpy(item.payload, payload, item.length);
  if (xQueueSend(logQueue, &item, 0) != pdPASS) ++logDrops;
}

void sendControl(const TxItem& item) {
  boat::Header header{}; header.version = boat::kVersion; header.type = static_cast<uint8_t>(item.type); header.length = item.length;
  header.sequence = ++txSequence; header.bootId = bootId; header.sourceUs = nowUs(); header.flags = 0;
  uint8_t encoded[boat::kMaxEncoded]{}; const size_t size = boat::encode(header, item.payload, encoded, sizeof(encoded));
  if (!size || controlUart.write(encoded, size) != size) { ++linkDrops; return; }
  ++txFrames; enqueueLog(header, item.payload);
}

void serviceControlTx() {
  TxItem item{};
  for (uint8_t i = 0; i < 8 && xQueueReceive(controlTxQueue, &item, 0) == pdTRUE; ++i) sendControl(item);
}

void sendHeartbeat() {
  boat::HeartbeatPayload h{millis(), txSequence, 0, 1, 0};
  enqueueControl(boat::Type::Heartbeat, &h, sizeof(h));
}

void sendGnssNav() {
  const auto& g = gnssRx.latest();
  boat::GnssNavV2Payload nav{}; nav.navSequence = ++navSequence; nav.fixSequence = navSequence;
  if (g.flags & gnss::FixValid) nav.flags |= boat::NavFixValid;
  if (g.flags & gnss::LatitudeValid) nav.flags |= boat::NavLatValid;
  if (g.flags & gnss::LongitudeValid) nav.flags |= boat::NavLonValid;
  if (g.flags & gnss::AltitudeValid) nav.flags |= boat::NavAltitudeValid;
  if (g.flags & gnss::SpeedValid) nav.flags |= boat::NavSpeedValid;
  if (g.flags & gnss::CourseValid) nav.flags |= boat::NavCourseValid;
  if (g.flags & gnss::HdopValid) nav.flags |= boat::NavHdopValid;
  nav.latitudeE7 = static_cast<int32_t>(g.latitude * 1e7); nav.longitudeE7 = static_cast<int32_t>(g.longitude * 1e7);
  nav.altitudeMm = static_cast<int32_t>(g.altitudeM * 1000.0f); nav.speedMmPerSec = static_cast<int32_t>(g.speedMps * 1000.0f);
  nav.courseE5Deg = static_cast<int32_t>(g.courseDeg * 100000.0f); nav.hdopCenti = static_cast<uint16_t>(g.hdop * 100.0f);
  nav.satellites = g.satellites; nav.fixType = g.fixType; nav.generatedUs = nowUs(); nav.measurementUs = g.lastValidFixUs;
  nav.sourceBootId = bootId; nav.canonicalCrc = boat::canonicalCrc(&nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
  enqueueControl(boat::Type::GnssNavV2, &nav, sizeof(nav));
}

void handleControlFrame(const boat::Frame& frame) {
  ++rxFrames; lastControlUs = nowUs(); enqueueLog(frame.header, frame.payload);
  if (static_cast<boat::Type>(frame.header.type) == boat::Type::BnoQuaternion && frame.header.length == sizeof(boat::BnoPayload)) {
    boat::BnoPayload p{}; memcpy(&p, frame.payload, sizeof(p));
    if (p.kind == 3) { portENTER_CRITICAL(&cacheMux); rotation.payload = p; rotation.receivedUs = lastControlUs; rotation.valid = true; portEXIT_CRITICAL(&cacheMux); }
  }
}

void serviceControlRx() {
  for (uint16_t i = 0; i < 512 && controlUart.available(); ++i) {
    boat::Frame frame{};
    if (decoder.feed(static_cast<uint8_t>(controlUart.read()), frame)) { frame.uartRxUs = nowUs(); handleControlFrame(frame); }
  }
}

void serviceGnss() {
  gnss::Sentence sentence{};
  for (uint16_t i = 0; i < kGnssReadBudgetBytes && gnssUart.available(); ++i) gnssRx.feed(static_cast<char>(gnssUart.read()), nowUs(), sentence);
  gnssRx.expire(nowUs());
}

String nextLogName() {
  if (!SD.exists(kLogDirectory)) SD.mkdir(kLogDirectory);
  for (uint16_t n = 1; n < 10000; ++n) { char path[32]; snprintf(path, sizeof(path), "%s/RUN%04u.BIN", kLogDirectory, n); if (!SD.exists(path)) return String(path); }
  return String();
}

void logTask(void*) {
  File file;
  for (;;) {
    if (logCommand == 1) {
      logCommand = 0; activeLogName = nextLogName();
      if (sdReady && activeLogName.length()) { file = SD.open(activeLogName, FILE_WRITE); logging = static_cast<bool>(file); }
    }
    LogItem item{};
    if (logging && xQueueReceive(logQueue, &item, pdMS_TO_TICKS(kLogTaskWakeMs)) == pdTRUE) {
      const size_t a = file.write(reinterpret_cast<const uint8_t*>(&item.header), sizeof(item.header));
      const size_t b = item.length ? file.write(item.payload, item.length) : 0;
      if (a != sizeof(item.header) || (item.length && b != item.length)) { ++logDrops; logging = false; file.close(); }
      else ++logRecords;
    }
    if (logCommand == 2) { logCommand = 0; if (file) { file.flush(); file.close(); } logging = false; }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void apiStatus() {
  RotationCache r{}; portENTER_CRITICAL(&cacheMux); r = rotation; portEXIT_CRITICAL(&cacheMux);
  const auto& g = gnssRx.latest(); const uint64_t now = nowUs();
  const uint32_t linkAge = lastControlUs ? static_cast<uint32_t>((now - lastControlUs) / 1000ULL) : UINT32_MAX;
  const uint32_t rotationAge = r.receivedUs ? static_cast<uint32_t>((now - r.receivedUs) / 1000ULL) : UINT32_MAX;
  char body[1150]; snprintf(body, sizeof(body),
    "{\"firmware\":\"%s\",\"physical_output_enabled\":false,\"sd\":\"%s\",\"logging\":%s,\"log_file\":\"%s\",\"records\":%lu,\"log_drops\":%lu,\"link\":{\"age_ms\":%lu,\"rx_frames\":%lu,\"tx_frames\":%lu,\"drops\":%lu,\"crc_errors\":%lu,\"cobs_errors\":%lu,\"length_errors\":%lu},\"gnss\":{\"receiving\":%s,\"fix\":%s,\"age_ms\":%lu,\"lat\":%.7f,\"lon\":%.7f,\"speed_mps\":%.3f,\"sats\":%u},\"rotation_vector\":{\"valid\":%s,\"age_ms\":%lu,\"accuracy\":%u,\"q\":[%.6f,%.6f,%.6f,%.6f],\"rpy_deg\":[%.3f,%.3f,%.3f]}}",
    kFirmwareVersion, sdReady ? "ready" : "unavailable", logging ? "true" : "false", activeLogName.c_str(), (unsigned long)logRecords, (unsigned long)logDrops,
    (unsigned long)linkAge, (unsigned long)rxFrames, (unsigned long)txFrames, (unsigned long)linkDrops, (unsigned long)decoder.crcErrors, (unsigned long)decoder.cobsErrors, (unsigned long)decoder.lengthErrors,
    gnssRx.receiving(now) ? "true" : "false", (g.flags & gnss::FixValid) ? "true" : "false", (unsigned long)(g.lastValidFixUs ? (now - g.lastValidFixUs) / 1000ULL : UINT32_MAX), g.latitude, g.longitude, g.speedMps, g.satellites,
    r.valid ? "true" : "false", (unsigned long)rotationAge, r.payload.accuracy, r.payload.v[3], r.payload.v[0], r.payload.v[1], r.payload.v[2], r.payload.v[4], r.payload.v[5], r.payload.v[6]);
  web.send(200, "application/json", body);
}

void apiLogStart() { if (!sdReady) { web.send(503, "application/json", "{\"error\":\"sd_unavailable\"}"); return; } logCommand = 1; web.send(202, "application/json", "{\"accepted\":true}"); }
void apiLogStop() { logCommand = 2; web.send(202, "application/json", "{\"accepted\":true}"); }
void controlRoute(boat::Type type) { enqueueControl(type); web.send(202, "application/json", "{\"accepted\":true}"); }

const char page[] PROGMEM = R"HTML(<!doctype html><meta name=viewport content="width=device-width,initial-scale=1"><style>body{font:15px system-ui;background:#101720;color:#edf3fa;margin:12px}.c{background:#1d2a38;border-radius:9px;padding:10px;margin:8px 0;white-space:pre-wrap}button{padding:9px;margin:2px}canvas{width:100%;height:130px;background:#080d13}</style><h2>通信側 XIAO / Rotation Vector</h2><div class=c id=s>loading</div><div class=c><button onclick="p('/api/log/start')">記録開始</button><button onclick="p('/api/log/stop')">記録停止</button><button onclick="p('/api/control/stop')">STOP</button><button onclick="p('/api/control/estop')">E-STOP</button></div><canvas id=g></canvas><script>let h=[];async function p(u){await fetch(u,{method:'POST'})}function d(){let w=g.width=g.clientWidth*devicePixelRatio,H=g.height=g.clientHeight*devicePixelRatio,c=g.getContext('2d');c.clearRect(0,0,w,H);c.strokeStyle='#79e5a0';c.beginPath();h.forEach((v,i)=>{let x=i*w/159,y=H/2-v/180*H*.42;i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke()}async function u(){try{let j=await(await fetch('/api/status',{cache:'no-store'})).json(),r=j.rotation_vector;s.textContent=`SD ${j.sd}, log ${j.logging}, records ${j.records}\nUART age ${j.link.age_ms} ms, frames RX/TX ${j.link.rx_frames}/${j.link.tx_frames}, drops ${j.link.drops}\nGNSS ${j.gnss.fix?'fix':'no fix'} age ${j.gnss.age_ms} ms, sats ${j.gnss.sats}\nRotation Vector ${r.valid?'valid':'invalid'} age ${r.age_ms} ms, accuracy ${r.accuracy}\nRoll/Pitch/Yaw ${r.rpy_deg.map(x=>x.toFixed(2)).join(' / ')} deg`;h.push(r.rpy_deg[2]);if(h.length>160)h.shift();d()}catch(e){s.textContent='更新エラー'}}setInterval(u,50);u()</script>)HTML";

void beginWeb() {
  WiFi.persistent(false); WiFi.mode(WIFI_AP); WiFi.softAP(kApSsid, kApPassword);
  web.on("/", HTTP_GET, [] { web.send(200, "text/html", page); }); web.on("/api/status", HTTP_GET, apiStatus);
  web.on("/api/log/start", HTTP_POST, apiLogStart); web.on("/api/log/stop", HTTP_POST, apiLogStop);
  web.on("/api/control/arm", HTTP_POST, [] { controlRoute(boat::Type::Arm); }); web.on("/api/control/disarm", HTTP_POST, [] { controlRoute(boat::Type::Disarm); });
  web.on("/api/control/start", HTTP_POST, [] { controlRoute(boat::Type::StartTest); }); web.on("/api/control/stop", HTTP_POST, [] { controlRoute(boat::Type::Stop); }); web.on("/api/control/estop", HTTP_POST, [] { controlRoute(boat::Type::Estop); });
  web.begin();
}
}  // namespace

void setup() {
  Serial.begin(115200); bootId = esp_random();
  controlTxQueue = xQueueCreate(48, sizeof(TxItem)); logQueue = xQueueCreate(48, sizeof(LogItem));
  controlUart.begin(kControlUartBaud, SERIAL_8N1, kControlUartRxPin, kControlUartTxPin);
  gnssUart.setRxBufferSize(kGnssUartRxBufferBytes); gnssUart.begin(kGnssBaud, SERIAL_8N1, kGnssRxPin, kGnssTxPin); gnssRx.begin(gnssUart);
  SPI.begin(kSdSckPin, kSdMisoPin, kSdMosiPin, kSdCsPin); sdReady = SD.begin(kSdCsPin, SPI);
  xTaskCreatePinnedToCore(logTask, "SdLog", 4096, nullptr, kLogTaskPriority, nullptr, 0); beginWeb();
  Serial.printf("%s %s boot=%lu SD=%d AP=%s IP=%s BNO=absent\n", kFirmwareName, kFirmwareVersion, (unsigned long)bootId, sdReady, kApSsid, WiFi.softAPIP().toString().c_str());
}

void loop() {
  serviceGnss(); serviceControlRx();
  const uint32_t now = millis(); if (now - lastNavMs >= kGnssNavIntervalMs) { lastNavMs = now; sendGnssNav(); }
  if (now - lastHeartbeatMs >= kControlHeartbeatIntervalMs) { lastHeartbeatMs = now; sendHeartbeat(); }
  serviceControlTx(); web.handleClient();
  if (now - lastDiagMs >= kDiagnosticIntervalMs) { lastDiagMs = now; Serial.printf("COMM sd=%d log=%d rec=%lu link_age_ms=%lu gnss=%d\n", sdReady, logging, (unsigned long)logRecords, (unsigned long)(lastControlUs ? (nowUs()-lastControlUs)/1000ULL : UINT32_MAX), gnssRx.receiving(nowUs())); }
  delay(1);
}
