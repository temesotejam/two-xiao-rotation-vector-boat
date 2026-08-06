#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <esp_timer.h>
#include <stddef.h>
#include "app_config.h"
#include "gnss_receiver.h"
#include "boat_protocol.h"

using namespace app_config;

HardwareSerial controlUart(1);
HardwareSerial gnssUart(2);
WebServer web(kHttpPort);
gnss::Receiver gnssRx;
boat::Decoder controlDecoder;
QueueHandle_t txQueue = nullptr;
QueueHandle_t logQueue = nullptr;

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
float clampf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }

struct TxItem {
  boat::Type type = boat::Type::Event;
  uint16_t length = 0;
  uint8_t payload[boat::kMaxPayload]{};
};

struct LogItem {
  uint64_t localUs = 0;
  boat::Header header{};
  uint16_t length = 0;
  uint8_t payload[boat::kMaxPayload]{};
};

struct Cache {
  boat::SystemHealthPayload health{};
  boat::ActuatorStatePayload actuator{};
  boat::ControlSnapshotPayload control{};
  boat::VescTelemetryPayload vesc{};
  boat::InaStatusPayload ina{};
  boat::CommandAckPayload ack{};
  boat::BnoPayload rotation{};
  uint64_t healthRxUs = 0;
  uint64_t actuatorRxUs = 0;
  uint64_t controlRxUs = 0;
  uint64_t vescRxUs = 0;
  uint64_t inaRxUs = 0;
  uint64_t ackRxUs = 0;
  uint64_t rotationRxUs = 0;
  bool healthValid = false;
  bool actuatorValid = false;
  bool controlValid = false;
  bool vescValid = false;
  bool inaValid = false;
  bool ackValid = false;
  bool rotationValid = false;
} cache;

struct ManualConfig {
  uint8_t enabledMask = boat::ManualLeft;
  float left = 0;
  float right = 0;
  float rear = 0;
  float propulsion = 0;
  bool configured = false;
  uint64_t lastSentUs = 0;
} manualConfig;

uint32_t bootId = 0;
uint32_t txSequence = 0;
uint32_t structuredCommandSequence = 0;
uint32_t requestSequence = 0;
uint32_t navSequence = 0;
uint32_t syncSequence = 0;
uint32_t rxFrames = 0;
uint32_t txFrames = 0;
uint32_t txDrops = 0;
uint32_t logDrops = 0;
uint32_t logRecords = 0;
uint32_t rxSequenceGaps = 0;
uint32_t previousRxSequence = 0;
uint64_t lastControlRxUs = 0;
uint64_t lastGnssNavUs = 0;
uint64_t lastHeartbeatUs = 0;
uint64_t lastSyncUs = 0;
uint64_t lastDiagnosticUs = 0;
int64_t controlMinusCommunicationUs = 0;
uint32_t syncRttUs = 0;
uint32_t syncUncertaintyUs = 0;
bool sdReady = false;
volatile bool logging = false;
volatile uint8_t logCommand = 0;
String activeLogName;
boat::ControlMode selectedMode = boat::ControlMode::Manual;

#include "modules/link_and_log.inc"
#include "modules/input_and_sync.inc"
#include "modules/web_service.inc"
#include "modules/runtime.inc"
