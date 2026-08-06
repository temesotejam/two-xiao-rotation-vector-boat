#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <freertos/queue.h>

namespace bno {
enum class EventType : uint8_t { Accel = 1, Gyro = 2, Rotation = 3, Magnetic = 4, LinearAcceleration = 5 };
struct Sample {
  EventType type = EventType::Accel;
  uint8_t accuracy = 0;
  uint8_t sequence = 0;
  uint64_t sensorUs = 0;
  uint64_t callbackUs = 0;
  uint64_t queuePushUs = 0;
  float v[7]{};
};
struct Latest {
  bool rotationValid = false;
  float qx = 0, qy = 0, qz = 0, qw = 1;
  float roll = 0, pitch = 0, yaw = 0;
  uint64_t rotationUs = 0;
  uint64_t lastUs = 0;
};
struct Metrics {
  uint32_t initCount = 0;
  uint32_t reinitCount = 0;
  uint32_t callbacks = 0;
  uint32_t decodeErrors = 0;
  uint32_t queueDrops = 0;
  uint32_t accelEvents = 0;
  uint32_t gyroEvents = 0;
  uint32_t rotationEvents = 0;
  uint32_t magneticEvents = 0;
  uint32_t linearEvents = 0;
  uint32_t maxPollUs = 0;
};
class Reader {
 public:
  bool begin();
  void poll(void (*output)(const Sample&));
  void recover();
  bool ready() const { return ready_; }
  const char* fault() const { return fault_; }
  const Latest& latest() const { return latest_; }
  Metrics metrics() const;
 private:
  struct QueuedEvent { sh2_SensorValue_t value{}; uint64_t rxUs = 0; uint64_t queuePushUs = 0; };
  static void sensorCallback(void* cookie, sh2_SensorEvent_t* event);
  void onSensorEvent(sh2_SensorEvent_t* event);
  bool init();
  bool enableReports();
  void handle(const QueuedEvent& event, void (*output)(const Sample&));
  void setFault(const char* value);
  Adafruit_BNO08x sensor_{-1};
  QueueHandle_t queue_ = nullptr;
  bool ready_ = false;
  uint8_t address_ = 0;
  uint32_t lastReinitMs_ = 0;
  char fault_[64] = "not initialized";
  Latest latest_{};
  mutable portMUX_TYPE metricsMux_ = portMUX_INITIALIZER_UNLOCKED;
  Metrics metrics_{};
};
}  // namespace bno
