void bnoEvent(const bno::Sample& sample) {
  boat::BnoPayload payload{};
  payload.kind = static_cast<uint8_t>(sample.type);
  payload.accuracy = sample.accuracy;
  payload.sequence = sample.sequence;
  payload.sensorUs = sample.sensorUs;
  payload.callbackUs = sample.callbackUs;
  payload.queuePushUs = sample.queuePushUs;
  memcpy(payload.v, sample.v, sizeof(payload.v));
  boat::Type type = boat::Type::BnoQuaternion;
  portENTER_CRITICAL(&sensorMux);
  if (sample.type == bno::EventType::Gyro) {
    imuState.gx = sample.v[0]; imuState.gy = sample.v[1]; imuState.gz = sample.v[2]; imuState.gyroUs = sample.callbackUs; imuState.gyroValid = true;
    type = boat::Type::BnoGyro;
  } else if (sample.type == bno::EventType::Accel) {
    type = boat::Type::BnoAccel;
  } else if (sample.type == bno::EventType::Magnetic) {
    type = boat::Type::BnoMagnetic;
  } else if (sample.type == bno::EventType::Rotation) {
    imuState.qx = sample.v[0]; imuState.qy = sample.v[1]; imuState.qz = sample.v[2]; imuState.qw = sample.v[3];
    imuState.roll = degToRad(sample.v[4]); imuState.pitch = degToRad(sample.v[5]); imuState.yaw = degToRad(sample.v[6]);
    imuState.accuracy = sample.accuracy; imuState.attitudeUs = sample.callbackUs; imuState.attitudeValid = true;
  }
  portEXIT_CRITICAL(&sensorMux);
  enqueue(type, &payload, sizeof(payload));
}

void bnoTask(void*) {
  for (;;) {
    const bool asserted = digitalRead(kBnoIntPin) == LOW;
    const bool notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kBnoTaskFallbackMs)) != 0;
    if (asserted || notified) imu.poll(bnoEvent);
    imu.recover();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void IRAM_ATTR bnoIntIsr() {
  BaseType_t woke = pdFALSE;
  if (bnoTaskHandle) vTaskNotifyGiveFromISR(bnoTaskHandle, &woke);
  if (woke) portYIELD_FROM_ISR();
}

float computeHeightM(const VL53L5CX_ResultsData& data) {
  const uint8_t indices[4] = {27, 28, 35, 36};
  float sum = 0;
  uint8_t count = 0;
  for (uint8_t index : indices) {
    const uint16_t mm = data.distance_mm[index];
    const uint8_t status = data.target_status[index];
    if (mm >= 30 && mm <= 4000 && (status == 5 || status == 6 || status == 9)) { sum += mm; ++count; }
  }
  return count ? (sum / count) / 1000.0f : NAN;
}

void serviceTof() {
  if (!tofReady || !tof.isDataReady() || !tof.getRangingData(&tofData)) return;
  const uint64_t t = nowUs();
  const float height = computeHeightM(tofData);
  tofState.valid = isfinite(height);
  tofState.heightM = height;
  tofState.timestampUs = t;
  ++tofState.frames;
  boat::TofFramePayload payload{};
  payload.timestampUs = t;
  memcpy(payload.distanceMm, tofData.distance_mm, sizeof(payload.distanceMm));
  memcpy(payload.targetStatus, tofData.target_status, sizeof(payload.targetStatus));
  enqueue(boat::Type::TofFrame, &payload, sizeof(payload));
}

void serviceIna() {
  const uint64_t t = nowUs();
  if (!inaReady || t - lastInaPollUs < kInaSampleIntervalUs) return;
  lastInaPollUs = t;
  InaSample sample{};
  if (!ina.read(sample)) return;
  powerState.valid = sample.valid;
  powerState.busVoltageV = sample.busV;
  powerState.currentA = sample.currentRegA;
  powerState.powerW = sample.powerRegW;
  powerState.timestampUs = sample.us ? sample.us : t;
  boat::InaStatusPayload payload{};
  payload.timestampUs = powerState.timestampUs;
  payload.ageUs = 0;
  payload.busVoltageV = sample.busV;
  payload.shuntVoltageV = sample.shuntV;
  payload.currentA = sample.currentRegA;
  payload.powerW = sample.powerRegW;
  payload.valid = sample.valid;
  payload.errorCode = sample.valid ? 0 : 1;
  enqueue(boat::Type::InaStatus, &payload, sizeof(payload));
}

void publishVesc() {
  boat::VescTelemetryPayload payload{};
  payload.timestampUs = lastVescResponseUs;
  payload.ageUs = 0;
  payload.inputVoltageV = vescValues.inputVoltageV;
  payload.motorCurrentA = vescValues.motorCurrentA;
  payload.inputCurrentA = vescValues.inputCurrentA;
  payload.duty = vescValues.duty;
  payload.erpm = vescValues.erpm;
  payload.mosTempC = vescValues.mosTempC;
  payload.motorTempC = vescValues.motorTempC;
  payload.tachometer = vescValues.tachometer;
  payload.valid = vescValues.valid;
  payload.fault = vescValues.fault;
  enqueue(boat::Type::VescTelemetry, &payload, sizeof(payload));
}

void serviceVescRx() {
  const uint64_t t = nowUs();
  while (vescUart.available()) {
    vesc::Frame frame{};
    vesc::ParseError error{};
    if (vescParser.feed(static_cast<uint8_t>(vescUart.read()), t, frame, error)) {
      if (frame.crcOk && frame.endOk && vesc::Parser::parseCurrentValues(frame, vescValues)) {
        lastVescResponseUs = t;
        publishVesc();
      }
    }
  }
  vesc::ParseError timeoutError{};
  vescParser.timeout(t, timeoutError);
  if (t - lastVescRequestUs >= static_cast<uint64_t>(kVescRequestIntervalMs) * 1000ULL) requestVescValues();
}

void geoToLocal(double latitude, double longitude, float& north, float& east) {
  constexpr double earth = 6378137.0;
  const double dLat = (latitude - navigationState.originLat) * PI / 180.0;
  const double dLon = (longitude - navigationState.originLon) * PI / 180.0;
  const double meanLat = (latitude + navigationState.originLat) * 0.5 * PI / 180.0;
  north = static_cast<float>(dLat * earth);
  east = static_cast<float>(dLon * earth * cos(meanLat));
}

bool updateNavigation(ControlResult& result) {
  if (!gnssState.valid || waypointCount == 0 || activeWaypoint >= waypointCount) return false;
  if (!navigationState.originSet) {
    navigationState.originSet = true;
    navigationState.originLat = gnssState.latitudeDeg;
    navigationState.originLon = gnssState.longitudeDeg;
  }
  geoToLocal(gnssState.latitudeDeg, gnssState.longitudeDeg, navigationState.northM, navigationState.eastM);
  float targetNorth = 0, targetEast = 0;
  geoToLocal(waypoints[activeWaypoint].latitudeDeg, waypoints[activeWaypoint].longitudeDeg, targetNorth, targetEast);
  const float dn = targetNorth - navigationState.northM;
  const float de = targetEast - navigationState.eastM;
  const float distance = sqrtf(dn * dn + de * de);
  if (distance <= waypointReachM) {
    if (activeWaypoint + 1 < waypointCount) {
      ++activeWaypoint;
      geoToLocal(waypoints[activeWaypoint].latitudeDeg, waypoints[activeWaypoint].longitudeDeg, targetNorth, targetEast);
    } else {
      result.waypointComplete = true;
      result.waypointDistance = distance;
      return true;
    }
  }
  float startNorth = 0, startEast = 0;
  if (activeWaypoint > 0) geoToLocal(waypoints[activeWaypoint - 1].latitudeDeg, waypoints[activeWaypoint - 1].longitudeDeg, startNorth, startEast);
  const float segN = targetNorth - startNorth;
  const float segE = targetEast - startEast;
  const float segLength = sqrtf(segN * segN + segE * segE);
  float aimNorth = targetNorth, aimEast = targetEast;
  if (segLength > 0.1f) {
    const float unitN = segN / segLength;
    const float unitE = segE / segLength;
    const float projection = (navigationState.northM - startNorth) * unitN + (navigationState.eastM - startEast) * unitE;
    const float aimAlong = clampf(projection + kLosLookaheadM, 0, segLength);
    aimNorth = startNorth + aimAlong * unitN;
    aimEast = startEast + aimAlong * unitE;
  }
  navigationState.targetBearing = atan2f(aimEast - navigationState.eastM, aimNorth - navigationState.northM);
  navigationState.distanceM = sqrtf((targetNorth - navigationState.northM) * (targetNorth - navigationState.northM) + (targetEast - navigationState.eastM) * (targetEast - navigationState.eastM));
  result.targetBearing = navigationState.targetBearing;
  result.waypointDistance = navigationState.distanceM;
  return true;
}

