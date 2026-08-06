#include "boat_protocol.h"
#include <string.h>

namespace boat {
uint32_t crc32(const uint8_t* data, size_t bytes) {
  uint32_t crc = 0xffffffffu;
  while (bytes--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
  }
  return ~crc;
}

static size_t cobsEncode(const uint8_t* input, size_t bytes, uint8_t* output) {
  size_t out = 0;
  size_t codePos = out++;
  uint8_t code = 1;
  for (size_t i = 0; i < bytes; ++i) {
    if (input[i] == 0) {
      output[codePos] = code;
      code = 1;
      codePos = out++;
    } else {
      output[out++] = input[i];
      if (++code == 0xff) {
        output[codePos] = code;
        code = 1;
        codePos = out++;
      }
    }
  }
  output[codePos] = code;
  return out;
}

size_t encode(const Header& header, const uint8_t* payload, uint8_t* output, size_t capacity) {
  const size_t rawBytes = sizeof(Header) + header.length + sizeof(uint32_t);
  if (header.length > kMaxPayload || rawBytes > kMaxRaw || capacity < kMaxEncoded) return 0;
  uint8_t raw[kMaxRaw]{};
  memcpy(raw, &header, sizeof(Header));
  if (header.length && payload) memcpy(raw + sizeof(Header), payload, header.length);
  const uint32_t crc = crc32(raw, sizeof(Header) + header.length);
  memcpy(raw + sizeof(Header) + header.length, &crc, sizeof(crc));
  size_t encoded = cobsEncode(raw, rawBytes, output);
  output[encoded++] = 0;
  return encoded;
}

bool Decoder::feed(uint8_t byte, Frame& frame) {
  if (byte != 0) {
    if (n_ >= sizeof(encoded_)) { n_ = 0; ++lengthErrors; }
    else encoded_[n_++] = byte;
    return false;
  }
  if (n_ == 0) return false;
  uint8_t raw[kMaxRaw]{};
  size_t decoded = 0;
  for (size_t i = 0; i < n_;) {
    const uint8_t code = encoded_[i++];
    if (code == 0 || i + static_cast<size_t>(code - 1) > n_) { ++cobsErrors; n_ = 0; return false; }
    for (uint8_t j = 1; j < code; ++j) {
      if (decoded >= sizeof(raw)) { ++lengthErrors; n_ = 0; return false; }
      raw[decoded++] = encoded_[i++];
    }
    if (code < 0xff && i < n_) {
      if (decoded >= sizeof(raw)) { ++lengthErrors; n_ = 0; return false; }
      raw[decoded++] = 0;
    }
  }
  n_ = 0;
  if (decoded < sizeof(Header) + sizeof(uint32_t)) { ++lengthErrors; return false; }
  memcpy(&frame.header, raw, sizeof(Header));
  if (frame.header.version != kVersion || frame.header.length > kMaxPayload || decoded != sizeof(Header) + frame.header.length + sizeof(uint32_t)) { ++lengthErrors; return false; }
  uint32_t supplied = 0;
  memcpy(&supplied, raw + sizeof(Header) + frame.header.length, sizeof(supplied));
  if (supplied != crc32(raw, sizeof(Header) + frame.header.length)) { ++crcErrors; return false; }
  if (frame.header.length) memcpy(frame.payload, raw + sizeof(Header), frame.header.length);
  return true;
}
}  // namespace boat
