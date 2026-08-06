#include <cassert>
#include <cstring>
#include <iostream>
#include "../control/lib/boat_protocol/src/boat_protocol.h"

int main() {
  boat::ManualCommandPayload command{};
  command.protocolVersion = 1;
  command.enabledMask = boat::ManualLeft | boat::ManualPropulsion;
  command.requestId = 42;
  command.commandSequence = 77;
  command.leftFrontWing = 0.25f;
  command.propulsion = 0.03f;
  command.canonicalCrc = boat::canonicalCrc(&command, offsetof(boat::ManualCommandPayload, canonicalCrc));

  boat::Header header{};
  header.version = boat::kVersion;
  header.type = static_cast<uint8_t>(boat::Type::ManualCommand);
  header.length = sizeof(command);
  header.sequence = 123;
  header.bootId = 456;
  header.sourceUs = 789;

  uint8_t encoded[boat::kMaxEncoded]{};
  const size_t bytes = boat::encode(header, reinterpret_cast<const uint8_t*>(&command), encoded, sizeof(encoded));
  assert(bytes > 0);

  boat::Decoder decoder;
  boat::Frame frame{};
  bool complete = false;
  for (size_t i = 0; i < bytes; ++i) complete = decoder.feed(encoded[i], frame) || complete;
  assert(complete);
  assert(frame.header.sequence == header.sequence);
  assert(frame.header.length == sizeof(command));
  boat::ManualCommandPayload decoded{};
  std::memcpy(&decoded, frame.payload, sizeof(decoded));
  assert(decoded.commandSequence == command.commandSequence);
  assert(decoded.canonicalCrc == boat::canonicalCrc(&decoded, offsetof(boat::ManualCommandPayload, canonicalCrc)));

  encoded[4] ^= 0x55;
  boat::Decoder damaged;
  complete = false;
  for (size_t i = 0; i < bytes; ++i) complete = damaged.feed(encoded[i], frame) || complete;
  assert(!complete);
  assert(damaged.crcErrors + damaged.cobsErrors + damaged.lengthErrors > 0);
  std::cout << "protocol test passed\n";
}
