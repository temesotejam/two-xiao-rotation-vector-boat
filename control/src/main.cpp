#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <math.h>
#include <stddef.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "app_config.h"
#include "bno_reader.h"
#include "ina226_reader.h"
#include "pca9685.h"
#include "vesc_protocol.h"
#include "boat_protocol.h"

using namespace app_config;

HardwareSerial linkUart(1);
HardwareSerial vescUart(2);
Pca9685 pca;
Ina226 ina;
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData{};
bno::Reader imu;
vesc::Parser vescParser;
vesc::Values vescValues{};
boat::Decoder linkDecoder;
QueueHandle_t txQueue = nullptr;
TaskHandle_t bnoTaskHandle = nullptr;
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

uint64_t nowUs() { return static_cast<uint64_t>(esp_timer_get_time()); }
float clampf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }
float wrapPi(float a) { while (a > PI) a -= 2.0f * PI; while (a < -PI) a += 2.0f * PI; return a; }
float degToRad(float d) { return d * PI / 180.0f; }

struct TxItem {
  boat::Type type = boat::Type::Event;
  uint16_t length = 0;
  uint8_t payload[boat::kMaxPayload]{};
};

struct ImuState {
  bool attitudeValid = false;
  bool gyroValid = false;
  uint8_t accuracy = 0;
  float qx = 0, qy = 0, qz = 0, qw = 1;
  float roll = 0, pitch = 0, yaw = 0;
  float gx = 0, gy = 0, gz = 0;
  uint64_t attitudeUs = 0;
  uint64_t gyroUs = 0;
} imuState;

struct TofState {
  bool valid = false;
  float heightM = NAN;
  uint64_t timestampUs = 0;
  uint32_t frames = 0;
} tofState;

struct PowerState {
  bool valid = false;
  float busVoltageV = NAN;
  float currentA = NAN;
  float powerW = NAN;
  uint64_t timestampUs = 0;
} powerState;

struct GnssState {
  bool valid = false;
  double latitudeDeg = 0;
  double longitudeDeg = 0;
  float speedMps = 0;
  float courseRad = 0;
  uint8_t satellites = 0;
  uint64_t measurementUs = 0;
  uint64_t receivedUs = 0;
  uint32_t sequence = 0;
} gnssState;

struct ManualState {
  uint8_t enabledMask = 0;
  float left = 0;
  float right = 0;
  float rear = 0;
  float propulsion = 0;
  uint32_t commandSequence = 0;
  uint64_t receivedUs = 0;
} manualState;

struct ControlResult {
  float left = 0;
  float right = 0;
  float rear = 0;
  float propulsion = 0;
  float targetBearing = 0;
  float courseError = 0;
  float waypointDistance = NAN;
  bool valid = false;
  bool waypointComplete = false;
} controlResult;

struct NavigationState {
  bool originSet = false;
  double originLat = 0;
  double originLon = 0;
  float northM = 0;
  float eastM = 0;
  float targetBearing = 0;
  float distanceM = NAN;
} navigationState;

boat::WaypointGeo waypoints[boat::kMaxWaypoints]{};
uint8_t waypointCount = 0;
uint8_t activeWaypoint = 0;
uint32_t waypointRevision = 0;
float waypointReachM = kWaypointReachM;

boat::SafetyState safetyState = boat::SafetyState::Boot;
boat::ControlMode controlMode = boat::ControlMode::Manual;
boat::StopReason stopReason = boat::StopReason::None;
float headingTargetRad = 0;
uint32_t bootId = 0;
uint32_t txSequence = 0;
uint32_t txFrames = 0;
uint32_t txDrops = 0;
uint32_t rxFrames = 0;
uint32_t rxSequenceGaps = 0;
uint32_t previousRxSequence = 0;
uint64_t lastHostHeartbeatUs = 0;
uint64_t lastControlUs = 0;
uint64_t lastInaPollUs = 0;
uint64_t lastVescResponseUs = 0;
uint64_t lastVescCommandUs = 0;
uint64_t lastVescRequestUs = 0;
uint64_t stallStartedUs = 0;
uint32_t lastFastTelemetryMs = 0;
uint32_t lastSlowTelemetryMs = 0;
uint32_t lastDiagnosticMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t controlCycles = 0;
uint32_t pwmWrites = 0;
uint32_t pwmErrors = 0;
uint8_t activeServoMask = 0;
uint16_t leftPulseUs = 0, rightPulseUs = 0, rearPulseUs = 0;
float targetDuty = 0;
float appliedDuty = 0;
bool motorRelayEnabled = false;
bool pcaReady = false;
bool inaReady = false;
bool tofReady = false;
int64_t controlMinusCommunicationUs = 0;
uint32_t timeSyncRttUs = 0;
uint32_t lastStructuredCommandSequence = 0;
uint32_t lastSimpleCommandFrameSequence = 0;

#include "modules/link_output.inc"
#include "modules/commands_and_link.inc"
#include "modules/sensors_and_navigation.inc"
#include "modules/control_and_actuators.inc"
#include "modules/telemetry_and_runtime.inc"
