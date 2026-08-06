#include "boat_protocol.h"

namespace boat {
uint32_t crc32(const uint8_t* data, size_t bytes) {
  uint32_t crc = 0xffffffff;
  while (bytes--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
  }
  return ~crc;
}

static size_t cobs(const uint8_t* input, size_t bytes, uint8_t* output) {
  size_t out = 0, codePos = 0;
  uint8_t code = 1;
  output[out++] = 0;
  for (size_t i = 0; i < bytes; ++i) {
    if (input[i] == 0) { output[codePos] = code; code = 1; codePos = out++; }
    else { output[out++] = input[i]; if (++code == 0xff) { output[codePos] = code; code = 1; codePos = out++; } }
  }
  output[codePos] = code;
  return out;
}

size_t encode(const Header& header, const uint8_t* payload, uint8_t* output, size_t capacity) {
  if (header.length > kMaxPayload || capacity < kMaxEncoded) return 0;
  uint8_t raw[kMaxRaw];
  memcpy(raw, &header, sizeof(header));
  if (header.length) memcpy(raw + sizeof(header), payload, header.length);
  const uint32_t crc = crc32(raw, sizeof(header) + header.length);
  memcpy(raw + sizeof(header) + header.length, &crc, sizeof(crc));
  size_t encoded = cobs(raw, sizeof(header) + header.length + sizeof(crc), output);
  output[encoded++] = 0;
  return encoded;
}

bool Decoder::feed(uint8_t byte, Frame& frame) {
  if (byte) {
    if (n_ >= sizeof(encoded_)) { n_ = 0; ++lengthErrors; }
    else encoded_[n_++] = byte;
    return false;
  }
  if (!n_) return false;
  uint8_t raw[kMaxRaw]; size_t decoded = 0;
  for (size_t i = 0; i < n_;) {
    const uint8_t code = encoded_[i++];
    if (!code || i + code - 1 > n_ + 1) { ++cobsErrors; n_ = 0; return false; }
    for (uint8_t j = 1; j < code && i < n_; ++j) raw[decoded++] = encoded_[i++];
    if (code < 0xff && i < n_) raw[decoded++] = 0;
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