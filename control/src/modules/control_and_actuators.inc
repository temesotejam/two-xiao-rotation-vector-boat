ControlResult computeControl(float dt) {
  ControlResult result{};
  ImuState s{};
  portENTER_CRITICAL(&sensorMux); s = imuState; portEXIT_CRITICAL(&sensorMux);
  const uint64_t t = nowUs();
  if (controlMode == boat::ControlMode::Manual) {
    result.left = manualState.left;
    result.right = manualState.right;
    result.rear = manualState.rear;
    result.propulsion = manualState.propulsion;
    result.valid = true;
    return result;
  }
  if (!imuFresh(t)) return result;
  const float heightTerm = tofFresh(t) ? kKpHeight * (kTargetHeightM - tofState.heightM) : 0.0f;
  const float pitchTerm = -kKpPitch * s.pitch - kKdPitch * s.gy;
  float rollTerm = -kKpRoll * s.roll - kKdRoll * s.gx;
  float targetYaw = headingTargetRad;
  float propulsion = manualState.propulsion;
  if (controlMode == boat::ControlMode::AutoWaypoint) {
    if (!updateNavigation(result) || result.waypointComplete) {
      result.valid = result.waypointComplete;
      return result;
    }
    targetYaw = result.targetBearing;
    propulsion = kAutoPropulsion;
  }
  const float yawError = wrapPi(targetYaw - s.yaw);
  float yawTerm = kKpYaw * yawError - kKdYaw * s.gz;
  const float pitchAbs = fabsf(s.pitch);
  if (pitchAbs > 0.45f) {
    const float scale = clampf((kCriticalPitchRad - pitchAbs) / (kCriticalPitchRad - 0.45f), 0, 1);
    propulsion *= scale;
    rollTerm *= scale;
    yawTerm *= scale;
  }
  const float common = pitchTerm + heightTerm;
  result.left = clampf(common + rollTerm, -1, 1);
  result.right = clampf(common - rollTerm, -1, 1);
  result.rear = clampf(yawTerm, -1, 1);
  result.propulsion = clampf(propulsion, -kVescMaxDuty, kVescMaxDuty);
  result.courseError = yawError;
  result.valid = true;
  (void)dt;
  return result;
}

uint16_t commandToPulse(float command, bool reversed) {
  command = clampf(reversed ? -command : command, -1, 1);
  const float pulse = command >= 0 ? kServoNeutralUs + command * (kServoMaxUs - kServoNeutralUs) : kServoNeutralUs + command * (kServoNeutralUs - kServoMinUs);
  return static_cast<uint16_t>(pulse + 0.5f);
}

uint16_t slewPulse(uint16_t current, uint16_t target, float dt) {
  if (!current) return target;
  const uint16_t step = static_cast<uint16_t>(fmaxf(1.0f, kServoRateUsPerSecond * dt));
  if (target > current) { const uint16_t delta = static_cast<uint16_t>(target - current); return static_cast<uint16_t>(current + (step < delta ? step : delta)); }
  const uint16_t delta = static_cast<uint16_t>(current - target); return static_cast<uint16_t>(current - (step < delta ? step : delta));
}

bool setServo(uint8_t channel, uint16_t& current, uint16_t target, float dt) {
  current = slewPulse(current, target, dt);
  if (kDryRun) return true;
  if (!pca.setPulse(channel, current)) { ++pwmErrors; return false; }
  ++pwmWrites;
  return true;
}

void disableServo(uint8_t channel, uint8_t maskBit) {
  if ((activeServoMask & maskBit) && pcaReady && !kDryRun) {
    if (!pca.setFullOff(channel)) ++pwmErrors;
  }
}

void applyActuators(const ControlResult& result, float dt) {
  if (safetyState != boat::SafetyState::Running || !result.valid) {
    fullOutputsOff();
    return;
  }
  uint8_t desiredMask = boat::ManualLeft | boat::ManualRight | boat::ManualRear;
  if (controlMode == boat::ControlMode::Manual) desiredMask = manualState.enabledMask & (boat::ManualLeft | boat::ManualRight | boat::ManualRear);
  disableServo(kLeftServoChannel, boat::ManualLeft & ~desiredMask);
  disableServo(kRightServoChannel, boat::ManualRight & ~desiredMask);
  disableServo(kRearServoChannel, boat::ManualRear & ~desiredMask);
  if (desiredMask & boat::ManualLeft) {
    if (!setServo(kLeftServoChannel, leftPulseUs, commandToPulse(result.left, kLeftServoReversed), dt)) { enterFault(boat::StopReason::PcaFailure); return; }
  } else leftPulseUs = 0;
  if (desiredMask & boat::ManualRight) {
    if (!setServo(kRightServoChannel, rightPulseUs, commandToPulse(result.right, kRightServoReversed), dt)) { enterFault(boat::StopReason::PcaFailure); return; }
  } else rightPulseUs = 0;
  if (desiredMask & boat::ManualRear) {
    if (!setServo(kRearServoChannel, rearPulseUs, commandToPulse(result.rear, kRearServoReversed), dt)) { enterFault(boat::StopReason::PcaFailure); return; }
  } else rearPulseUs = 0;
  activeServoMask = desiredMask;

  bool propulsionSelected = controlMode != boat::ControlMode::Manual || (manualState.enabledMask & boat::ManualPropulsion);
  targetDuty = propulsionSelected ? result.propulsion : 0;
  const uint64_t t = nowUs();
  const float voltage = monitoredVoltage(t);
  const float current = monitoredCurrent(t);
  if (isfinite(voltage) && voltage < kLowVoltageLimitV) targetDuty *= clampf((voltage - kCriticalVoltageV) / (kLowVoltageLimitV - kCriticalVoltageV), 0, 1);
  if (isfinite(current) && current > kOverCurrentLimitA) targetDuty *= clampf((kCriticalCurrentA - current) / (kCriticalCurrentA - kOverCurrentLimitA), 0, 1);
  if (fabsf(targetDuty) > 0.0005f && !propulsionHealthy(t)) targetDuty = 0;
  const float rate = fabsf(targetDuty) > fabsf(appliedDuty) ? kVescRisePerSecond : kVescFallPerSecond;
  const float step = rate * dt;
  if (targetDuty > appliedDuty) appliedDuty = fminf(targetDuty, appliedDuty + step);
  else appliedDuty = fmaxf(targetDuty, appliedDuty - step);
}

void serviceVescCommand() {
  const uint64_t t = nowUs();
  if (t - lastVescCommandUs < static_cast<uint64_t>(kVescCommandIntervalMs) * 1000ULL) return;
  float command = safetyState == boat::SafetyState::Running ? appliedDuty : 0;
  if (fabsf(command) > 0.0005f && !propulsionHealthy(t)) command = 0;
  if (!sendVescDuty(command)) enterFault(boat::StopReason::VescWriteFailure);
}

void serviceSafety() {
  const uint64_t t = nowUs();
  if (safetyState == boat::SafetyState::Boot || safetyState == boat::SafetyState::Disarmed || safetyState == boat::SafetyState::EStop || safetyState == boat::SafetyState::Fault) return;
  if (!linkFresh(t)) { enterFault(boat::StopReason::LinkTimeout); return; }
  if (controlMode == boat::ControlMode::Manual && t - manualState.receivedUs > kManualCommandTimeoutUs) { enterFault(boat::StopReason::ManualTimeout); return; }
  if (controlMode != boat::ControlMode::Manual && !imuFresh(t)) { enterFault(boat::StopReason::ImuInvalid); return; }
  if (controlMode == boat::ControlMode::AutoWaypoint && !gnssFresh(t)) { enterFault(boat::StopReason::GnssInvalid); return; }
  ImuState s{}; portENTER_CRITICAL(&sensorMux); s = imuState; portEXIT_CRITICAL(&sensorMux);
  if (imuFresh(t) && (fabsf(s.pitch) > kCriticalPitchRad || fabsf(s.roll) > kCriticalRollRad)) { enterFault(boat::StopReason::DangerousAttitude); return; }
  if (vescFresh(t) && vescValues.fault != 0) { enterFault(boat::StopReason::VescFault); return; }
  const float voltage = monitoredVoltage(t);
  const float current = monitoredCurrent(t);
  if (isfinite(voltage) && voltage <= kCriticalVoltageV) { enterFault(boat::StopReason::LowVoltage); return; }
  if (isfinite(current) && current >= kCriticalCurrentA) { enterFault(boat::StopReason::OverCurrent); return; }
  if (fabsf(appliedDuty) >= kStallDuty && vescFresh(t) && fabsf(vescValues.motorCurrentA) >= kStallCurrentA && fabsf(vescValues.erpm) < kStallErpm) {
    if (!stallStartedUs) stallStartedUs = t;
    if (t - stallStartedUs >= kStallTripUs) { enterFault(boat::StopReason::Stall); return; }
  } else stallStartedUs = 0;
}

