#include "bno_reader.h"
#include "app_config.h"
#include <esp_timer.h>
#include <math.h>
#include <string.h>

using namespace app_config;
namespace {
TwoWire bnoWire(1);
uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
uint8_t kindFor(uint8_t id) {
  if (id == SH2_ACCELEROMETER) return 1;
  if (id == SH2_GYROSCOPE_CALIBRATED) return 2;
  if (id == SH2_ROTATION_VECTOR) return 3;
  if (id == SH2_MAGNETIC_FIELD_CALIBRATED) return 4;
  if (id == SH2_LINEAR_ACCELERATION) return 5;
  return 0;
}
void quaternionToEuler(bno::Latest& value) {
  const float norm = sqrtf(value.qx * value.qx + value.qy * value.qy + value.qz * value.qz + value.qw * value.qw);
  if (norm < 1e-6f) return;
  const float qx = value.qx / norm, qy = value.qy / norm, qz = value.qz / norm, qw = value.qw / norm;
  value.roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / PI;
  const float sinPitch = fmaxf(-1.0f, fminf(1.0f, 2.0f * (qw * qy - qz * qx)));
  value.pitch = asinf(sinPitch) * 180.0f / PI;
  value.yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / PI;
}
}

namespace bno {
void Reader::setFault(const char* value) { snprintf(fault_, sizeof(fault_), "%s", value); }
Metrics Reader::metrics() const { Metrics copy{}; portENTER_CRITICAL(&metricsMux_); copy = metrics_; portEXIT_CRITICAL(&metricsMux_); return copy; }

bool Reader::enableReports() {
  bool ok = true;
  ok = sensor_.enableReport(SH2_ACCELEROMETER, kAccelGyroIntervalUs) && ok;
  ok = sensor_.enableReport(SH2_GYROSCOPE_CALIBRATED, kAccelGyroIntervalUs) && ok;
  ok = sensor_.enableReport(SH2_ROTATION_VECTOR, kRotationIntervalUs) && ok;
  ok = sensor_.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, kMagneticIntervalUs) && ok;
  ok = sensor_.enableReport(SH2_LINEAR_ACCELERATION, kRotationIntervalUs) && ok;
  return ok;
}

bool Reader::init() {
  ready_ = false;
  if (!queue_) { setFault("event queue unavailable"); return false; }
  xQueueReset(queue_);
  bnoWire.begin(kBnoSdaPin, kBnoSclPin, kBnoI2cHz);
  bnoWire.setTimeOut(20);
  uint8_t address = 0;
  const uint8_t candidates[] = {kBnoAddress, kBnoAlternateAddress};
  for (uint8_t candidate : candidates) {
    bnoWire.beginTransmission(candidate);
    if (bnoWire.endTransmission() == 0) { address = candidate; break; }
  }
  if (!address) { setFault("BNO08X not detected"); return false; }
  if (!sensor_.begin_I2C(address, &bnoWire)) { setFault("begin_I2C failed"); return false; }
  if (sh2_setSensorCallback(sensorCallback, this) != SH2_OK) { setFault("callback failed"); return false; }
  if (!enableReports()) { setFault("enable reports failed"); return false; }
  address_ = address;
  ready_ = true;
  latest_.lastUs = nowUs();
  portENTER_CRITICAL(&metricsMux_); ++metrics_.initCount; portEXIT_CRITICAL(&metricsMux_);
  setFault("none");
  return true;
}

bool Reader::begin() {
  pinMode(kBnoRstPin, OUTPUT);
  digitalWrite(kBnoRstPin, LOW);
  delay(10);
  digitalWrite(kBnoRstPin, HIGH);
  delay(100);
  pinMode(kBnoIntPin, INPUT_PULLUP);
  queue_ = xQueueCreate(kBnoEventQueueDepth, sizeof(QueuedEvent));
  if (!queue_) { setFault("queue allocation failed"); return false; }
  return init();
}

void Reader::sensorCallback(void* cookie, sh2_SensorEvent_t* event) {
  if (cookie) static_cast<Reader*>(cookie)->onSensorEvent(event);
}

void Reader::onSensorEvent(sh2_SensorEvent_t* event) {
  QueuedEvent queued{};
  if (sh2_decodeSensorEvent(&queued.value, event) != SH2_OK) {
    portENTER_CRITICAL(&metricsMux_); ++metrics_.decodeErrors; portEXIT_CRITICAL(&metricsMux_);
    return;
  }
  queued.rxUs = nowUs();
  queued.queuePushUs = nowUs();
  portENTER_CRITICAL(&metricsMux_); ++metrics_.callbacks; portEXIT_CRITICAL(&metricsMux_);
  if (!queue_ || xQueueSend(queue_, &queued, 0) != pdPASS) {
    portENTER_CRITICAL(&metricsMux_); ++metrics_.queueDrops; portEXIT_CRITICAL(&metricsMux_);
  }
}

void Reader::handle(const QueuedEvent& event, void (*output)(const Sample&)) {
  const uint8_t kind = kindFor(event.value.sensorId);
  if (!kind) return;
  Sample sample{};
  sample.accuracy = event.value.status & 3;
  sample.sequence = event.value.sequence;
  sample.sensorUs = event.value.timestamp;
  sample.callbackUs = event.rxUs;
  sample.queuePushUs = event.queuePushUs;
  if (kind == 1) {
    sample.type = EventType::Accel;
    sample.v[0] = event.value.un.accelerometer.x; sample.v[1] = event.value.un.accelerometer.y; sample.v[2] = event.value.un.accelerometer.z;
    portENTER_CRITICAL(&metricsMux_); ++metrics_.accelEvents; portEXIT_CRITICAL(&metricsMux_);
  } else if (kind == 2) {
    sample.type = EventType::Gyro;
    sample.v[0] = event.value.un.gyroscope.x; sample.v[1] = event.value.un.gyroscope.y; sample.v[2] = event.value.un.gyroscope.z;
    portENTER_CRITICAL(&metricsMux_); ++metrics_.gyroEvents; portEXIT_CRITICAL(&metricsMux_);
  } else if (kind == 3) {
    sample.type = EventType::Rotation;
    sample.v[0] = event.value.un.rotationVector.i; sample.v[1] = event.value.un.rotationVector.j; sample.v[2] = event.value.un.rotationVector.k; sample.v[3] = event.value.un.rotationVector.real;
    latest_.qx = sample.v[0]; latest_.qy = sample.v[1]; latest_.qz = sample.v[2]; latest_.qw = sample.v[3];
    quaternionToEuler(latest_);
    sample.v[4] = latest_.roll; sample.v[5] = latest_.pitch; sample.v[6] = latest_.yaw;
    latest_.rotationValid = true; latest_.rotationUs = event.rxUs;
    portENTER_CRITICAL(&metricsMux_); ++metrics_.rotationEvents; portEXIT_CRITICAL(&metricsMux_);
  } else if (kind == 4) {
    sample.type = EventType::Magnetic;
    sample.v[0] = event.value.un.magneticField.x; sample.v[1] = event.value.un.magneticField.y; sample.v[2] = event.value.un.magneticField.z;
    portENTER_CRITICAL(&metricsMux_); ++metrics_.magneticEvents; portEXIT_CRITICAL(&metricsMux_);
  } else {
    sample.type = EventType::LinearAcceleration;
    sample.v[0] = event.value.un.linearAcceleration.x; sample.v[1] = event.value.un.linearAcceleration.y; sample.v[2] = event.value.un.linearAcceleration.z;
    portENTER_CRITICAL(&metricsMux_); ++metrics_.linearEvents; portEXIT_CRITICAL(&metricsMux_);
  }
  latest_.lastUs = event.rxUs;
  if (output) output(sample);
}

void Reader::poll(void (*output)(const Sample&)) {
  if (!ready_) return;
  if (sensor_.wasReset()) {
    if (sh2_setSensorCallback(sensorCallback, this) != SH2_OK || !enableReports()) {
      ready_ = false;
      setFault("reset reconfigure failed");
      return;
    }
  }
  const uint64_t started = nowUs();
  uint8_t calls = 0;
  do { sh2_service(); ++calls; } while (digitalRead(kBnoIntPin) == LOW && calls < kBnoServiceCallBudget);
  QueuedEvent event{};
  while (xQueueReceive(queue_, &event, 0) == pdTRUE) handle(event, output);
  const uint32_t elapsed = static_cast<uint32_t>(nowUs() - started);
  portENTER_CRITICAL(&metricsMux_); if (elapsed > metrics_.maxPollUs) metrics_.maxPollUs = elapsed; portEXIT_CRITICAL(&metricsMux_);
}

void Reader::recover() {
  const uint32_t ms = millis();
  if (ready_ && latest_.lastUs && ms - static_cast<uint32_t>(latest_.lastUs / 1000ULL) <= kBnoNoDataTimeoutMs) return;
  if (ms - lastReinitMs_ < kReinitIntervalMs) return;
  lastReinitMs_ = ms;
  portENTER_CRITICAL(&metricsMux_); ++metrics_.reinitCount; portEXIT_CRITICAL(&metricsMux_);
  setFault(ready_ ? "data timeout" : "init retry");
  init();
}
}  // namespace bno
