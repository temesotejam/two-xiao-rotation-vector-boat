#include <Arduino.h>
#include <esp_timer.h>
#include "app_config.h"
#include "bno_reader.h"
#include "ina226_reader.h"
#include "pca9685.h"
#include "vesc_protocol.h"
#include <SparkFun_VL53L5CX_Library.h>
#include "boat_protocol.h"

using namespace app_config;
namespace {
HardwareSerial linkUart(1), vescUart(2);
Pca9685 pca; Ina226 ina; SparkFun_VL53L5CX tof; VL53L5CX_ResultsData tofData{};
vesc::Parser vescParser; vesc::Values vescValues{};
bool pcaReady=false, inaReady=false, tofReady=false; uint64_t lastInaUs=0, lastTofUs=0, lastVescUs=0;
bno::Reader imu;
QueueHandle_t txQueue = nullptr;
TaskHandle_t bnoTaskHandle = nullptr;
boat::Decoder decoder;
uint32_t bootId = 0, txSequence = 0, txDrops = 0, rxFrames = 0, txFrames = 0;
volatile uint32_t lastHostHeartbeatMs = 0;
uint32_t lastDiagnosticMs = 0;

enum class SafetyState : uint8_t { Boot = 0, Disarmed = 1, ArmedIdle = 2, Running = 3, EStop = 4, Fault = 5 };
volatile SafetyState safety = SafetyState::Boot;

struct TxItem { boat::Type type; uint16_t length; uint8_t payload[boat::kMaxPayload]; };
static_assert(!kActuatorOutputCompileEnable, "two_xiao_rotation_vector_shadow must not compile physical output");

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
const char* stateName(SafetyState s) { switch (s) { case SafetyState::Boot:return "BOOT"; case SafetyState::Disarmed:return "DISARMED"; case SafetyState::ArmedIdle:return "ARMED_IDLE"; case SafetyState::Running:return "RUNNING"; case SafetyState::EStop:return "E_STOP"; default:return "FAULT"; } }
void setState(SafetyState next) { if (next != safety) { safety = next; Serial.printf("STATE %s\n", stateName(next)); } }

bool enqueue(boat::Type type, const void* payload, size_t length) {
  if (!txQueue || length > boat::kMaxPayload) return false;
  TxItem item{}; item.type = type; item.length = static_cast<uint16_t>(length);
  if (length) memcpy(item.payload, payload, length);
  if (xQueueSend(txQueue, &item, 0) != pdPASS) { ++txDrops; return false; }
  return true;
}

void sendItem(const TxItem& item) {
  boat::Header h{}; h.version = boat::kVersion; h.type = static_cast<uint8_t>(item.type); h.length = item.length;
  h.sequence = ++txSequence; h.bootId = bootId; h.sourceUs = nowUs(); h.flags = 0;
  uint8_t encoded[boat::kMaxEncoded]{}; const size_t n = boat::encode(h, item.payload, encoded, sizeof(encoded));
  if (!n) { ++txDrops; return; }
  const size_t written = linkUart.write(encoded, n);
  if (written != n) ++txDrops; else ++txFrames;
}

void sendHeartbeat() {
  boat::HeartbeatPayload h{millis(), txSequence, static_cast<uint8_t>(safety), 1, 0};
  enqueue(boat::Type::Heartbeat, &h, sizeof(h));
}

void sendAck(const boat::Frame& frame, uint8_t disposition, uint16_t reason) {
  boat::CommandAckPayload ack{}; ack.commandId = frame.header.sequence; ack.commandType = frame.header.type;
  ack.disposition = disposition; ack.safetyState = static_cast<uint8_t>(safety); ack.dryRun = 1;
  ack.receivedUs = frame.uartRxUs; ack.appliedUs = nowUs(); ack.reason = reason;
  enqueue(boat::Type::CommandAck, &ack, sizeof(ack));
}

void txTask(void*) {
  uint32_t lastHeartbeatMs = 0;
  for (;;) {
    if (millis() - lastHeartbeatMs >= 100) { lastHeartbeatMs = millis(); sendHeartbeat(); }
    TxItem item{};
    if (xQueueReceive(txQueue, &item, pdMS_TO_TICKS(5)) == pdTRUE) sendItem(item);
  }
}

void bnoEvent(const bno::Sample& sample) {
  boat::BnoPayload p{}; p.kind = static_cast<uint8_t>(sample.type); p.accuracy = sample.accuracy; p.sequence = sample.sequence;
  p.sensorUs = sample.sensorUs; p.callbackUs = sample.callbackUs; p.queuePushUs = sample.queuePushUs;
  memcpy(p.v, sample.v, sizeof(p.v));
  boat::Type type = boat::Type::BnoQuaternion;
  if (sample.type == bno::EventType::Accel) type = boat::Type::BnoAccel;
  else if (sample.type == bno::EventType::Gyro) type = boat::Type::BnoGyro;
  else if (sample.type == bno::EventType::Magnetic) type = boat::Type::BnoMagnetic;
  enqueue(type, &p, sizeof(p));
}

void bnoTask(void*) {
  for (;;) {
    const bool asserted = digitalRead(kBnoIntPin) == LOW;
    const bool notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kBnoTaskFallbackMs)) != 0;
    if (asserted || notified) imu.poll(bnoEvent);
    imu.recover();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void IRAM_ATTR bnoIntIsr() {
  BaseType_t woke = pdFALSE;
  if (bnoTaskHandle) vTaskNotifyGiveFromISR(bnoTaskHandle, &woke);
  if (woke) portYIELD_FROM_ISR();
}

void handleFrame(const boat::Frame& frame) {
  ++rxFrames;
  const auto type = static_cast<boat::Type>(frame.header.type);
  if (type == boat::Type::Heartbeat) { lastHostHeartbeatMs = millis(); return; }
  if (type == boat::Type::GnssNavV2 && frame.header.length == sizeof(boat::GnssNavV2Payload)) {
    boat::GnssNavV2Payload nav{}; memcpy(&nav, frame.payload, sizeof(nav));
    if (nav.canonicalCrc == boat::canonicalCrc(&nav, offsetof(boat::GnssNavV2Payload, canonicalCrc))) return;
    return;
  }
  if (type == boat::Type::Arm && safety == SafetyState::Disarmed) { setState(SafetyState::ArmedIdle); sendAck(frame, 0, 0); return; }
  if (type == boat::Type::StartTest && safety == SafetyState::ArmedIdle) { setState(SafetyState::Running); sendAck(frame, 0, 0); return; }
  if (type == boat::Type::Disarm || type == boat::Type::Stop) { setState(SafetyState::Disarmed); sendAck(frame, 0, 0); return; }
  if (type == boat::Type::Estop) { setState(SafetyState::EStop); sendAck(frame, 0, 0); return; }
  if (type == boat::Type::ClearEstop && safety == SafetyState::EStop) { setState(SafetyState::Disarmed); sendAck(frame, 0, 0); return; }
  sendAck(frame, 1, 1);
}

void serviceLink() {
  for (uint16_t i = 0; i < 512 && linkUart.available(); ++i) {
    boat::Frame frame{};
    if (decoder.feed(static_cast<uint8_t>(linkUart.read()), frame)) { frame.uartRxUs = nowUs(); handleFrame(frame); }
  }
}

void pollPeripheralTelemetry() {
  const uint64_t t = nowUs();
  if (inaReady && t - lastInaUs >= 20000ULL) { lastInaUs = t; InaSample sample{}; if (ina.read(sample) && sample.valid) { struct __attribute__((packed)) { int16_t shunt,current; uint16_t bus,power,mask; uint32_t i2cUs; } p{sample.rawShunt,sample.rawCurrent,sample.rawBus,sample.rawPower,sample.mask,sample.i2cUs}; enqueue(boat::Type::InaSample,&p,sizeof(p)); } }
  if (tofReady && tof.isDataReady() && tof.getRangingData(&tofData)) { lastTofUs=t; uint8_t p[196]{}; memcpy(p,&lastTofUs,4); memcpy(p+4,tofData.distance_mm,128); memcpy(p+132,tofData.target_status,64); enqueue(boat::Type::TofFrame,p,sizeof(p)); }
  while (vescUart.available()) { vesc::Frame frame{}; vesc::ParseError error{}; if (vescParser.feed(static_cast<uint8_t>(vescUart.read()),t,frame,error) && frame.crcOk && frame.endOk && vesc::Parser::parseCurrentValues(frame,vescValues)) { lastVescUs=t; enqueue(boat::Type::VescStatus,&vescValues,sizeof(vescValues)); } }
}

void checkLinkHealth() {
  const uint32_t last = lastHostHeartbeatMs;
  if (last && millis() - last > kLinkFailSafeTimeoutMs && safety != SafetyState::Fault) setState(SafetyState::Fault);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  bootId = esp_random();
  txQueue = xQueueCreate(48, sizeof(TxItem));
  Wire.begin(kPeripheralSdaPin, kPeripheralSclPin, kPeripheralI2cHz); Wire.setTimeOut(20);
  pcaReady = pca.begin(); if (pcaReady) pca.allOff();
  ina.profile = InaProfile::Balanced; inaReady = ina.begin();
  tofReady = tof.begin(kTofAddress, Wire) && tof.setResolution(64) && tof.setRangingFrequency(10) && tof.setRangingMode(SF_VL53L5CX_RANGING_MODE::AUTONOMOUS) && tof.setIntegrationTime(50) && tof.startRanging();
  vescUart.begin(kVescUartBaud, SERIAL_8N1, kVescRxPin, kVescTxPin);
  linkUart.begin(kLinkBaud, SERIAL_8N1, kLinkRxPin, kLinkTxPin);
  imu.begin();
  xTaskCreatePinnedToCore(txTask, "TwoXiaoTx", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(bnoTask, "RotationVector", 6144, nullptr, 3, &bnoTaskHandle, 0);
  attachInterrupt(digitalPinToInterrupt(kBnoIntPin), bnoIntIsr, FALLING);
  setState(SafetyState::Disarmed);
  Serial.printf("two_xiao_rotation_vector_shadow boot=%lu bno=%d physical_output=0\n", (unsigned long)bootId, imu.ready());
}

void loop() {
  serviceLink(); checkLinkHealth(); pollPeripheralTelemetry();
  if (millis() - lastDiagnosticMs >= 1000) {
    lastDiagnosticMs = millis(); const auto& r = imu.latest(); const auto m = imu.metrics();
    const uint32_t age = r.rotationUs ? static_cast<uint32_t>(nowUs() - r.rotationUs) / 1000UL : UINT32_MAX;
    Serial.printf("DIRECT_RV state=%s ready=%d valid=%d age_ms=%lu rpy_deg=%.2f/%.2f/%.2f rx=%lu tx=%lu drop=%lu rv=%lu gyro=%lu accel=%lu\n",
      stateName(safety), imu.ready(), r.rotationValid, (unsigned long)age, r.roll, r.pitch, r.yaw,
      (unsigned long)rxFrames, (unsigned long)txFrames, (unsigned long)txDrops,
      (unsigned long)m.rotationEvents, (unsigned long)m.gyroEvents, (unsigned long)m.accelEvents, pcaReady, inaReady, tofReady, vescValues.valid);
  }
  delay(1);
}

