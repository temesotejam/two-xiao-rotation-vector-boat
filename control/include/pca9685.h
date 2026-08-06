#pragma once
#include <Arduino.h>

struct PcaChannel {
  uint16_t on = 0;
  uint16_t off = 0;
  bool fullOff = false;
};

class Pca9685 {
 public:
  bool begin();
  bool allOff();
  bool setFullOff(uint8_t channel);
  bool setPulse(uint8_t channel, uint16_t pulseUs, PcaChannel* actual = nullptr);
  bool readChannel(uint8_t channel, PcaChannel& output);
  bool detected = false;
  uint8_t mode1 = 0;
  uint8_t mode2 = 0;
  uint8_t prescale = 0;
  float actualHz = 0;
  uint32_t errors = 0;
  uint32_t readbackErrors = 0;
 private:
  bool ack();
  bool write8(uint8_t reg, uint8_t value);
  bool writeN(uint8_t reg, const uint8_t* data, size_t length);
  bool read8(uint8_t reg, uint8_t& value);
};
