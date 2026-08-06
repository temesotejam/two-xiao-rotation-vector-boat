void sendHeartbeat() {
  boat::HeartbeatPayload heartbeat{millis(), txSequence, static_cast<uint8_t>(safetyState), static_cast<uint8_t>(kDryRun), 0};
  enqueue(boat::Type::Heartbeat, &heartbeat, sizeof(heartbeat));
}

void sendFastTelemetry() {
  boat::ActuatorStatePayload actuator{};
  actuator.timestampUs = nowUs();
  actuator.pwmWrites = pwmWrites;
  actuator.pwmErrors = pwmErrors;
  actuator.leftPulseUs = leftPulseUs;
  actuator.rightPulseUs = rightPulseUs;
  actuator.rearPulseUs = rearPulseUs;
  actuator.motorRelayEnabled = motorRelayEnabled;
  actuator.enabledMask = activeServoMask | ((fabsf(appliedDuty) > 0.0005f) ? boat::ManualPropulsion : 0);
  actuator.targetDuty = targetDuty;
  actuator.appliedDuty = appliedDuty;
  actuator.pcaReady = pcaReady;
  actuator.outputsEnabled = safetyState == boat::SafetyState::Running;
  actuator.safetyState = static_cast<uint8_t>(safetyState);
  actuator.controlMode = static_cast<uint8_t>(controlMode);
  enqueue(boat::Type::ActuatorState, &actuator, sizeof(actuator));
}

void sendSlowTelemetry() {
  const uint64_t t = nowUs();
  ImuState s{}; portENTER_CRITICAL(&sensorMux); s = imuState; portEXIT_CRITICAL(&sensorMux);
  boat::SystemHealthPayload health{};
  health.timestampUs = t;
  if (pcaReady) health.flags |= 1u << 0;
  if (imuFresh(t)) health.flags |= 1u << 1;
  if (tofFresh(t)) health.flags |= 1u << 2;
  if (gnssFresh(t)) health.flags |= 1u << 3;
  if (inaFresh(t)) health.flags |= 1u << 4;
  if (vescFresh(t)) health.flags |= 1u << 5;
  if (linkFresh(t)) health.flags |= 1u << 6;
  if (motorRelayEnabled) health.flags |= 1u << 7;
  health.imuAgeUs = s.attitudeUs ? static_cast<uint32_t>(t - s.attitudeUs) : UINT32_MAX;
  health.tofAgeUs = tofState.timestampUs ? static_cast<uint32_t>(t - tofState.timestampUs) : UINT32_MAX;
  health.gnssAgeUs = gnssState.receivedUs ? static_cast<uint32_t>(t - gnssState.receivedUs) : UINT32_MAX;
  health.powerAgeUs = powerState.timestampUs ? static_cast<uint32_t>(t - powerState.timestampUs) : UINT32_MAX;
  health.vescAgeUs = lastVescResponseUs ? static_cast<uint32_t>(t - lastVescResponseUs) : UINT32_MAX;
  health.safetyState = static_cast<uint8_t>(safetyState);
  health.controlMode = static_cast<uint8_t>(controlMode);
  health.stopReason = static_cast<uint8_t>(stopReason);
  enqueue(boat::Type::SystemHealth, &health, sizeof(health));

  boat::ControlSnapshotPayload snapshot{};
  snapshot.timestampUs = t;
  snapshot.cycle = controlCycles;
  snapshot.waypointRevision = waypointRevision;
  snapshot.latitudeDeg = gnssState.latitudeDeg;
  snapshot.longitudeDeg = gnssState.longitudeDeg;
  if (waypointCount && activeWaypoint < waypointCount) {
    snapshot.targetLatitudeDeg = waypoints[activeWaypoint].latitudeDeg;
    snapshot.targetLongitudeDeg = waypoints[activeWaypoint].longitudeDeg;
  }
  snapshot.speedMps = gnssState.speedMps;
  snapshot.localNorthM = navigationState.northM;
  snapshot.localEastM = navigationState.eastM;
  snapshot.targetBearingRad = controlResult.targetBearing;
  snapshot.courseErrorRad = controlResult.courseError;
  snapshot.waypointDistanceM = controlResult.waypointDistance;
  snapshot.rollRad = s.roll;
  snapshot.pitchRad = s.pitch;
  snapshot.yawRad = s.yaw;
  snapshot.rollRateRadS = s.gx;
  snapshot.pitchRateRadS = s.gy;
  snapshot.yawRateRadS = s.gz;
  snapshot.heightM = tofState.heightM;
  snapshot.leftCommand = controlResult.left;
  snapshot.rightCommand = controlResult.right;
  snapshot.rearCommand = controlResult.rear;
  snapshot.propulsionCommand = controlResult.propulsion;
  snapshot.gnssValid = gnssFresh(t);
  snapshot.imuValid = imuFresh(t);
  snapshot.tofValid = tofFresh(t);
  snapshot.outputValid = controlResult.valid;
  snapshot.activeWaypoint = activeWaypoint;
  snapshot.waypointCount = waypointCount;
  snapshot.safetyState = static_cast<uint8_t>(safetyState);
  snapshot.controlMode = static_cast<uint8_t>(controlMode);
  enqueue(boat::Type::ControlSnapshot, &snapshot, sizeof(snapshot));
}

void setup() {
  pinMode(kMotorRelayPin, OUTPUT);
  setRelay(false);
  Serial.begin(115200);
  delay(100);
  bootId = esp_random();
  txQueue = xQueueCreate(kLinkTxQueueDepth, sizeof(TxItem));
  Wire.begin(kPeripheralSdaPin, kPeripheralSclPin, kPeripheralI2cHz);
  Wire.setTimeOut(20);
  pcaReady = pca.begin();
  if (pcaReady) pca.allOff();
  ina.profile = InaProfile::Balanced;
  inaReady = ina.begin();
  tofReady = tof.begin(kTofAddress, Wire) && tof.setResolution(64) && tof.setRangingFrequency(kTofFrequencyHz) &&
             tof.setRangingMode(SF_VL53L5CX_RANGING_MODE::AUTONOMOUS) && tof.setIntegrationTime(50) && tof.startRanging();
  vescUart.begin(kVescUartBaud, SERIAL_8N1, kVescRxPin, kVescTxPin);
  linkUart.begin(kLinkBaud, SERIAL_8N1, kLinkRxPin, kLinkTxPin);
  imu.begin();
  xTaskCreatePinnedToCore(txTask, "LinkTx", 4096, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(bnoTask, "Bno", 8192, nullptr, 3, &bnoTaskHandle, 0);
  attachInterrupt(digitalPinToInterrupt(kBnoIntPin), bnoIntIsr, FALLING);
  transitionTo(boat::SafetyState::Disarmed, boat::StopReason::None);
  Serial.printf("%s %s boot=%lu dry_run=%d pca=%d ina=%d tof=%d bno=%d\n", kFirmwareName, kFirmwareVersion,
                (unsigned long)bootId, kDryRun, pcaReady, inaReady, tofReady, imu.ready());
}

void loop() {
  serviceLink();
  serviceVescRx();
  serviceIna();
  serviceTof();
  serviceSafety();
  const uint64_t t = nowUs();
  if (t - lastControlUs >= kControlIntervalUs) {
    const float dt = lastControlUs ? (t - lastControlUs) / 1000000.0f : kControlIntervalUs / 1000000.0f;
    lastControlUs = t;
    ++controlCycles;
    controlResult = computeControl(dt);
    if (controlResult.waypointComplete && safetyState == boat::SafetyState::Running) {
      transitionTo(boat::SafetyState::Disarmed, boat::StopReason::WaypointComplete);
    } else {
      applyActuators(controlResult, dt);
    }
  }
  serviceVescCommand();
  const uint32_t ms = millis();
  if (ms - lastHeartbeatMs >= 100) { lastHeartbeatMs = ms; sendHeartbeat(); }
  if (ms - lastFastTelemetryMs >= kTelemetryFastIntervalMs) { lastFastTelemetryMs = ms; sendFastTelemetry(); }
  if (ms - lastSlowTelemetryMs >= kTelemetrySlowIntervalMs) { lastSlowTelemetryMs = ms; sendSlowTelemetry(); }
  if (ms - lastDiagnosticMs >= kDiagnosticIntervalMs) {
    lastDiagnosticMs = ms;
    Serial.printf("CONTROL state=%u mode=%u reason=%u link=%d imu=%d gnss=%d tof=%d vesc=%d relay=%d duty=%.3f/%.3f rx=%lu tx=%lu drop=%lu gaps=%lu sync_rtt=%lu\n",
      static_cast<unsigned>(safetyState), static_cast<unsigned>(controlMode), static_cast<unsigned>(stopReason), linkFresh(t), imuFresh(t),
      gnssFresh(t), tofFresh(t), vescFresh(t), motorRelayEnabled, targetDuty, appliedDuty, (unsigned long)rxFrames,
      (unsigned long)txFrames, (unsigned long)txDrops, (unsigned long)rxSequenceGaps, (unsigned long)timeSyncRttUs);
  }
  delay(1);
}

