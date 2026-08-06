#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif

namespace boat {
constexpr uint8_t kVersion = 2;
constexpr size_t kMaxPayload = 768;
constexpr size_t kMaxRaw = 800;
constexpr size_t kMaxEncoded = 820;
constexpr uint8_t kMaxWaypoints = 16;

enum class Type : uint8_t {
  BnoAccel = 2,
  BnoGyro = 3,
  BnoQuaternion = 4,
  TofFrame = 5,
  InaStatus = 6,
  VescTelemetry = 7,
  ActuatorState = 8,
  SystemHealth = 9,
  Event = 10,
  TimeSyncReply = 11,
  CommandAck = 17,
  TimeSyncRequest = 18,
  LinkStatistics = 19,
  BnoMagnetic = 20,
  ControlSnapshot = 22,
  Heartbeat = 32,
  Arm = 33,
  Disarm = 34,
  Start = 35,
  Stop = 36,
  Estop = 37,
  ClearEstop = 38,
  GnssNavV2 = 59,
  TimeSyncEstimate = 60,
  WaypointSet = 66,
  WaypointAck = 67,
  ControlModeCommand = 68,
  ManualCommand = 69,
  HeadingTarget = 70
};

enum class SafetyState : uint8_t { Boot = 0, Disarmed = 1, ArmedIdle = 2, Running = 3, EStop = 4, Fault = 5 };
enum class ControlMode : uint8_t { Manual = 0, AttitudeAssist = 1, HeadingHold = 2, AutoWaypoint = 3 };
enum class StopReason : uint8_t {
  None = 0, UserStop = 1, Estop = 2, LinkTimeout = 3, ManualTimeout = 4, ImuInvalid = 5,
  GnssInvalid = 6, VescInvalid = 7, VescFault = 8, LowVoltage = 9, OverCurrent = 10,
  Stall = 11, DangerousAttitude = 12, InvalidCommand = 13, PcaFailure = 14,
  VescWriteFailure = 15, WaypointComplete = 16
};

enum ManualOutputMask : uint8_t {
  ManualLeft = 1u << 0,
  ManualRight = 1u << 1,
  ManualRear = 1u << 2,
  ManualPropulsion = 1u << 3,
  ManualAll = ManualLeft | ManualRight | ManualRear | ManualPropulsion
};

struct __attribute__((packed)) Header { uint8_t version, type; uint16_t length; uint32_t sequence, bootId; uint64_t sourceUs; uint16_t flags; };
struct Frame { Header header{}; uint8_t payload[kMaxPayload]{}; uint64_t uartRxUs = 0; };
struct __attribute__((packed)) BnoPayload { uint8_t kind, accuracy, sequence, reserved; uint64_t sensorUs, callbackUs, queuePushUs; float v[7]; };
struct __attribute__((packed)) TofFramePayload { uint64_t timestampUs; uint16_t distanceMm[64]; uint8_t targetStatus[64]; };
struct __attribute__((packed)) InaStatusPayload { uint64_t timestampUs; uint32_t ageUs; float busVoltageV, shuntVoltageV, currentA, powerW; uint8_t valid, errorCode; uint16_t reserved; };
struct __attribute__((packed)) VescTelemetryPayload { uint64_t timestampUs; uint32_t ageUs; float inputVoltageV, motorCurrentA, inputCurrentA, duty, erpm, mosTempC, motorTempC; int32_t tachometer; uint8_t valid, fault, reserved[2]; };
struct __attribute__((packed)) ActuatorStatePayload { uint64_t timestampUs; uint32_t pwmWrites, pwmErrors; uint16_t leftPulseUs, rightPulseUs, rearPulseUs; uint8_t motorRelayEnabled, enabledMask; float targetDuty, appliedDuty; uint8_t pcaReady, outputsEnabled, safetyState, controlMode; };
struct __attribute__((packed)) SystemHealthPayload { uint64_t timestampUs; uint32_t flags, imuAgeUs, tofAgeUs, gnssAgeUs, powerAgeUs, vescAgeUs; uint8_t safetyState, controlMode, stopReason, reserved; };
struct __attribute__((packed)) ControlSnapshotPayload {
  uint64_t timestampUs; uint32_t cycle, waypointRevision; double latitudeDeg, longitudeDeg, targetLatitudeDeg, targetLongitudeDeg;
  float speedMps, localNorthM, localEastM, targetBearingRad, courseErrorRad, waypointDistanceM;
  float rollRad, pitchRad, yawRad, rollRateRadS, pitchRateRadS, yawRateRadS, heightM;
  float leftCommand, rightCommand, rearCommand, propulsionCommand;
  uint8_t gnssValid, imuValid, tofValid, outputValid, activeWaypoint, waypointCount, safetyState, controlMode;
};
struct __attribute__((packed)) HeartbeatPayload { uint32_t uptimeMs, sequence; uint8_t safetyState, dryRun; uint16_t reserved; };
struct __attribute__((packed)) GnssNavV2Payload {
  uint32_t navSequence, fixSequence, flags, utcCentiseconds;
  int32_t latitudeE7, longitudeE7, altitudeMm, speedMmPerSec, courseE5Deg;
  uint16_t hdopCenti, satellites; uint8_t fixType, reserved[3];
  uint64_t generatedUs, measurementUs; uint32_t sourceBootId, canonicalCrc;
};
constexpr uint32_t NavFixValid = 1u << 0, NavNewFix = 1u << 1, NavLatValid = 1u << 2, NavLonValid = 1u << 3,
  NavAltitudeValid = 1u << 4, NavSpeedValid = 1u << 5, NavCourseValid = 1u << 6, NavHdopValid = 1u << 7;
struct __attribute__((packed)) TimeSyncRequestPayload { uint32_t sequence; uint64_t t1Us; };
struct __attribute__((packed)) TimeSyncReplyPayload { uint32_t sequence; uint64_t t1Us, t2Us, t3Us; };
struct __attribute__((packed)) TimeSyncEstimatePayload { uint32_t sequence; int64_t controlMinusCommunicationUs; uint32_t rttUs, uncertaintyUs; uint64_t updatedUs; };
struct __attribute__((packed)) CommandAckPayload { uint32_t requestId, commandSequence; uint8_t commandType, disposition, safetyState, controlMode; uint16_t reason, reserved; uint64_t receivedUs, appliedUs; uint32_t canonicalCrc; };
struct __attribute__((packed)) ControlModeCommandPayload { uint8_t protocolVersion, mode; uint16_t reserved; uint32_t requestId, commandSequence; uint64_t sourceUs; uint32_t canonicalCrc; };
struct __attribute__((packed)) ManualCommandPayload { uint8_t protocolVersion, enabledMask; uint16_t reserved; uint32_t requestId, commandSequence; uint64_t sourceUs; float leftFrontWing, rightFrontWing, rearYaw, propulsion; uint32_t canonicalCrc; };
struct __attribute__((packed)) HeadingTargetPayload { uint8_t protocolVersion, reserved[3]; uint32_t requestId, commandSequence; uint64_t sourceUs; float targetYawRad; uint32_t canonicalCrc; };
struct __attribute__((packed)) WaypointGeo { double latitudeDeg, longitudeDeg; };
struct __attribute__((packed)) WaypointSetPayload { uint32_t requestId, revision; uint8_t action, count, reserved[2]; float reachRadiusM; WaypointGeo points[kMaxWaypoints]; uint32_t canonicalCrc; };
struct __attribute__((packed)) WaypointAckPayload { uint32_t requestId, revision; uint8_t status, reason, activeIndex, count; uint32_t canonicalCrc; };
struct __attribute__((packed)) EventPayload { uint64_t timestampUs; uint16_t code, detail; int32_t value; };

uint32_t crc32(const uint8_t* data, size_t bytes);
inline uint32_t canonicalCrc(const void* value, size_t bytes) { return crc32(static_cast<const uint8_t*>(value), bytes); }
size_t encode(const Header& header, const uint8_t* payload, uint8_t* output, size_t capacity);
class Decoder {
 public:
  bool feed(uint8_t byte, Frame& frame);
  uint32_t crcErrors = 0, cobsErrors = 0, lengthErrors = 0;
 private:
  uint8_t encoded_[kMaxEncoded]{};
  size_t n_ = 0;
};
}  // namespace boat
