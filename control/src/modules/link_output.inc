bool enqueue(boat::Type type, const void* payload = nullptr, size_t length = 0) {
  if (!txQueue || length > boat::kMaxPayload) return false;
  TxItem item{};
  item.type = type;
  item.length = static_cast<uint16_t>(length);
  if (length && payload) memcpy(item.payload, payload, length);
  if (xQueueSend(txQueue, &item, 0) != pdPASS) {
    ++txDrops;
    return false;
  }
  return true;
}

void sendItem(const TxItem& item) {
  boat::Header header{};
  header.version = boat::kVersion;
  header.type = static_cast<uint8_t>(item.type);
  header.length = item.length;
  header.sequence = ++txSequence;
  header.bootId = bootId;
  header.sourceUs = nowUs();
  uint8_t encoded[boat::kMaxEncoded]{};
  const size_t count = boat::encode(header, item.payload, encoded, sizeof(encoded));
  if (!count || linkUart.write(encoded, count) != count) {
    ++txDrops;
    return;
  }
  ++txFrames;
}

void txTask(void*) {
  TxItem item{};
  for (;;) {
    if (xQueueReceive(txQueue, &item, pdMS_TO_TICKS(5)) == pdTRUE) sendItem(item);
  }
}

void setRelay(bool enabled) {
  if (kDryRun) enabled = false;
  motorRelayEnabled = enabled;
  const bool level = kMotorRelayActiveHigh ? enabled : !enabled;
  digitalWrite(kMotorRelayPin, level ? HIGH : LOW);
}

size_t makeVescPacket(const uint8_t* payload, size_t payloadLength, uint8_t* output, size_t capacity) {
  if (!payload || !payloadLength || payloadLength > 255 || capacity < payloadLength + 5) return 0;
  size_t n = 0;
  output[n++] = 2;
  output[n++] = static_cast<uint8_t>(payloadLength);
  memcpy(output + n, payload, payloadLength);
  n += payloadLength;
  const uint16_t crc = vesc::Parser::crc16(payload, payloadLength);
  output[n++] = static_cast<uint8_t>(crc >> 8);
  output[n++] = static_cast<uint8_t>(crc & 0xff);
  output[n++] = 3;
  return n;
}

bool sendVescPayload(const uint8_t* payload, size_t payloadLength) {
  uint8_t packet[32]{};
  const size_t count = makeVescPacket(payload, payloadLength, packet, sizeof(packet));
  return count && vescUart.write(packet, count) == count;
}

bool sendVescDuty(float duty) {
  duty = clampf(duty, -kVescMaxDuty, kVescMaxDuty);
  const bool moving = fabsf(duty) > 0.0005f;
  if (moving && kDryRun) duty = 0;
  if (moving) setRelay(true);
  const int32_t raw = static_cast<int32_t>(duty * 100000.0f);
  uint8_t payload[5] = {
    vesc::kCommSetDuty,
    static_cast<uint8_t>((raw >> 24) & 0xff),
    static_cast<uint8_t>((raw >> 16) & 0xff),
    static_cast<uint8_t>((raw >> 8) & 0xff),
    static_cast<uint8_t>(raw & 0xff)
  };
  const bool ok = kDryRun ? true : sendVescPayload(payload, sizeof(payload));
  lastVescCommandUs = nowUs();
  if (!moving || !ok) setRelay(false);
  return ok;
}

void requestVescValues() {
  const uint8_t command = vesc::kCommGetValues;
  if (sendVescPayload(&command, 1)) lastVescRequestUs = nowUs();
}

void fullOutputsOff() {
  const bool motorWasActive = motorRelayEnabled || fabsf(targetDuty) > 0.0005f || fabsf(appliedDuty) > 0.0005f;
  targetDuty = 0;
  appliedDuty = 0;
  if (motorWasActive) sendVescDuty(0); else setRelay(false);
  if (pcaReady && activeServoMask) {
    if (!pca.allOff()) ++pwmErrors;
  }
  activeServoMask = 0;
  leftPulseUs = rightPulseUs = rearPulseUs = 0;
}

void sendEvent(uint16_t code, uint16_t detail = 0, int32_t value = 0) {
  boat::EventPayload event{nowUs(), code, detail, value};
  enqueue(boat::Type::Event, &event, sizeof(event));
}

void transitionTo(boat::SafetyState next, boat::StopReason reason = boat::StopReason::None) {
  if (next == safetyState && reason == stopReason) return;
  if (next == boat::SafetyState::Disarmed || next == boat::SafetyState::EStop || next == boat::SafetyState::Fault) fullOutputsOff();
  safetyState = next;
  stopReason = reason;
  sendEvent(100 + static_cast<uint16_t>(next), static_cast<uint16_t>(reason));
  Serial.printf("STATE %u reason=%u\n", static_cast<unsigned>(next), static_cast<unsigned>(reason));
}

void enterFault(boat::StopReason reason) {
  if (safetyState == boat::SafetyState::EStop) return;
  transitionTo(boat::SafetyState::Fault, reason);
}

bool linkFresh(uint64_t t) { return lastHostHeartbeatUs && t - lastHostHeartbeatUs <= static_cast<uint64_t>(kLinkHeartbeatTimeoutMs) * 1000ULL; }
bool imuFresh(uint64_t t) {
  ImuState s{};
  portENTER_CRITICAL(&sensorMux); s = imuState; portEXIT_CRITICAL(&sensorMux);
  return s.attitudeValid && s.gyroValid && t - s.attitudeUs <= kImuAttitudeStaleUs && t - s.gyroUs <= kImuGyroStaleUs;
}
bool gnssFresh(uint64_t t) { return gnssState.valid && t - gnssState.receivedUs <= kGnssStaleUs; }
bool tofFresh(uint64_t t) { return tofState.valid && t - tofState.timestampUs <= kTofStaleUs; }
bool vescFresh(uint64_t t) { return vescValues.valid && lastVescResponseUs && t - lastVescResponseUs <= kVescStaleUs; }
bool inaFresh(uint64_t t) { return powerState.valid && t - powerState.timestampUs <= kPowerStaleUs; }

float monitoredVoltage(uint64_t t) {
  if (inaFresh(t)) return powerState.busVoltageV;
  if (vescFresh(t)) return vescValues.inputVoltageV;
  return NAN;
}
float monitoredCurrent(uint64_t t) {
  if (inaFresh(t)) return fabsf(powerState.currentA);
  if (vescFresh(t)) return fabsf(vescValues.inputCurrentA);
  return NAN;
}
bool propulsionHealthy(uint64_t t) {
  if (!vescFresh(t) || vescValues.fault != 0) return false;
  const float voltage = monitoredVoltage(t);
  return isfinite(voltage) && voltage > kCriticalVoltageV;
}

