#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif

namespace boat {
constexpr uint8_t kVersion = 1;
constexpr size_t kMaxPayload = 768, kMaxRaw = 800, kMaxEncoded = 820;

enum class Type : uint8_t {
  BnoAccel = 2, BnoGyro = 3, BnoQuaternion = 4, TofFrame = 5,
  InaSample = 6, VescStatus = 7, CommandAck = 17, BnoMagnetic = 20,
  Heartbeat = 32, Arm = 33, Disarm = 34, StartTest = 35, Stop = 36,
  Estop = 37, ClearEstop = 38, GnssNavV2 = 59
};

struct __attribute__((packed)) Header {
  uint8_t version, type;
  uint16_t length;
  uint32_t sequence, bootId;
  uint64_t sourceUs;
  uint16_t flags;
};
struct Frame {
  Header header{};
  uint8_t payload[kMaxPayload]{};
  uint64_t uartRxUs = 0, logQueueUs = 0, sdTaskUs = 0;
};
struct __attribute__((packed)) BnoPayload {
  uint8_t kind, accuracy, sequence, reserved;
  uint64_t sensorUs, callbackUs, queuePushUs;
  float v[7];
};
struct __attribute__((packed)) HeartbeatPayload {
  uint32_t uptimeMs, sequence;
  uint8_t safetyState, dryRun;
  uint16_t reserved;
};
struct __attribute__((packed)) CommandAckPayload {
  uint32_t commandId;
  uint8_t commandType, disposition, safetyState, dryRun;
  uint64_t receivedUs, appliedUs;
  uint16_t reason, reserved;
};
struct __attribute__((packed)) GnssNavV2Payload {
  uint32_t navSequence, fixSequence, flags, utcCentiseconds;
  int32_t latitudeE7, longitudeE7, altitudeMm, speedMmPerSec, courseE5Deg;
  uint16_t hdopCenti, satellites;
  uint8_t fixType, reserved[3];
  uint64_t generatedUs, measurementUs;
  uint32_t sourceBootId, canonicalCrc;
};
constexpr uint32_t NavFixValid = 1u << 0;
constexpr uint32_t NavLatValid = 1u << 2;
constexpr uint32_t NavLonValid = 1u << 3;
constexpr uint32_t NavAltitudeValid = 1u << 4;
constexpr uint32_t NavSpeedValid = 1u << 5;
constexpr uint32_t NavCourseValid = 1u << 6;
constexpr uint32_t NavHdopValid = 1u << 7;

uint32_t crc32(const uint8_t*, size_t);
inline uint32_t canonicalCrc(const void* value, size_t bytes) {
  return crc32(static_cast<const uint8_t*>(value), bytes);
}
size_t encode(const Header&, const uint8_t*, uint8_t*, size_t);
class Decoder {
 public:
  bool feed(uint8_t, Frame&);
  uint32_t crcErrors = 0, cobsErrors = 0, lengthErrors = 0;
 private:
  uint8_t encoded_[kMaxEncoded]{};
  size_t n_ = 0;
};
}  // namespace boat