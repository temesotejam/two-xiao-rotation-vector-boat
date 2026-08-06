void handleTimeSyncReply(const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::TimeSyncReplyPayload)) return;
  boat::TimeSyncReplyPayload reply{};
  memcpy(&reply, frame.payload, sizeof(reply));
  const uint64_t t4 = frame.uartRxUs;
  if (t4 < reply.t1Us || reply.t3Us < reply.t2Us) return;
  const int64_t a = static_cast<int64_t>(reply.t2Us) - static_cast<int64_t>(reply.t1Us);
  const int64_t b = static_cast<int64_t>(reply.t3Us) - static_cast<int64_t>(t4);
  controlMinusCommunicationUs = (a + b) / 2;
  const uint64_t total = t4 - reply.t1Us;
  const uint64_t remote = reply.t3Us - reply.t2Us;
  syncRttUs = static_cast<uint32_t>(total > remote ? total - remote : 0);
  syncUncertaintyUs = syncRttUs / 2;
  boat::TimeSyncEstimatePayload estimate{};
  estimate.sequence = reply.sequence;
  estimate.controlMinusCommunicationUs = controlMinusCommunicationUs;
  estimate.rttUs = syncRttUs;
  estimate.uncertaintyUs = syncUncertaintyUs;
  estimate.updatedUs = t4;
  enqueueControl(boat::Type::TimeSyncEstimate, &estimate, sizeof(estimate));
}

void handleControlFrame(const boat::Frame& frame) {
  ++rxFrames;
  lastControlRxUs = frame.uartRxUs;
  if (previousRxSequence && frame.header.sequence > previousRxSequence + 1) rxSequenceGaps += frame.header.sequence - previousRxSequence - 1;
  previousRxSequence = frame.header.sequence;
  enqueueLog(frame.header, frame.payload, frame.uartRxUs, false);
  const boat::Type type = static_cast<boat::Type>(frame.header.type);
  if (type == boat::Type::SystemHealth && frame.header.length == sizeof(cache.health)) {
    memcpy(&cache.health, frame.payload, sizeof(cache.health)); cache.healthValid = true; cache.healthRxUs = frame.uartRxUs;
  } else if (type == boat::Type::ActuatorState && frame.header.length == sizeof(cache.actuator)) {
    memcpy(&cache.actuator, frame.payload, sizeof(cache.actuator)); cache.actuatorValid = true; cache.actuatorRxUs = frame.uartRxUs;
  } else if (type == boat::Type::ControlSnapshot && frame.header.length == sizeof(cache.control)) {
    memcpy(&cache.control, frame.payload, sizeof(cache.control)); cache.controlValid = true; cache.controlRxUs = frame.uartRxUs;
  } else if (type == boat::Type::VescTelemetry && frame.header.length == sizeof(cache.vesc)) {
    memcpy(&cache.vesc, frame.payload, sizeof(cache.vesc)); cache.vescValid = true; cache.vescRxUs = frame.uartRxUs;
  } else if (type == boat::Type::InaStatus && frame.header.length == sizeof(cache.ina)) {
    memcpy(&cache.ina, frame.payload, sizeof(cache.ina)); cache.inaValid = true; cache.inaRxUs = frame.uartRxUs;
  } else if (type == boat::Type::CommandAck && frame.header.length == sizeof(cache.ack)) {
    boat::CommandAckPayload ack{}; memcpy(&ack, frame.payload, sizeof(ack));
    if (ack.canonicalCrc == boat::canonicalCrc(&ack, offsetof(boat::CommandAckPayload, canonicalCrc))) {
      cache.ack = ack; cache.ackValid = true; cache.ackRxUs = frame.uartRxUs;
    }
  } else if (type == boat::Type::BnoQuaternion && frame.header.length == sizeof(cache.rotation)) {
    boat::BnoPayload rotation{}; memcpy(&rotation, frame.payload, sizeof(rotation));
    if (rotation.kind == 3) { cache.rotation = rotation; cache.rotationValid = true; cache.rotationRxUs = frame.uartRxUs; }
  } else if (type == boat::Type::TimeSyncReply) {
    handleTimeSyncReply(frame);
  }
}

void serviceControlRx() {
  for (uint16_t i = 0; i < 768 && controlUart.available(); ++i) {
    boat::Frame frame{};
    if (controlDecoder.feed(static_cast<uint8_t>(controlUart.read()), frame)) {
      frame.uartRxUs = nowUs();
      handleControlFrame(frame);
    }
  }
}

void serviceGnss() {
  gnss::Sentence sentence{};
  for (uint16_t i = 0; i < kGnssReadBudgetBytes && gnssUart.available(); ++i) {
    gnssRx.feed(static_cast<char>(gnssUart.read()), nowUs(), sentence);
  }
  gnssRx.expire(nowUs());
}

void sendGnssNav() {
  const auto& g = gnssRx.latest();
  boat::GnssNavV2Payload nav{};
  nav.navSequence = ++navSequence;
  nav.fixSequence = navSequence;
  if (g.flags & gnss::FixValid) nav.flags |= boat::NavFixValid;
  if (g.flags & gnss::LatitudeValid) nav.flags |= boat::NavLatValid;
  if (g.flags & gnss::LongitudeValid) nav.flags |= boat::NavLonValid;
  if (g.flags & gnss::AltitudeValid) nav.flags |= boat::NavAltitudeValid;
  if (g.flags & gnss::SpeedValid) nav.flags |= boat::NavSpeedValid;
  if (g.flags & gnss::CourseValid) nav.flags |= boat::NavCourseValid;
  if (g.flags & gnss::HdopValid) nav.flags |= boat::NavHdopValid;
  nav.latitudeE7 = static_cast<int32_t>(g.latitude * 1e7);
  nav.longitudeE7 = static_cast<int32_t>(g.longitude * 1e7);
  nav.altitudeMm = static_cast<int32_t>(g.altitudeM * 1000.0f);
  nav.speedMmPerSec = static_cast<int32_t>(g.speedMps * 1000.0f);
  nav.courseE5Deg = static_cast<int32_t>(g.courseDeg * 100000.0f);
  nav.hdopCenti = static_cast<uint16_t>(clampf(g.hdop * 100.0f, 0, 65535));
  nav.satellites = g.satellites;
  nav.fixType = g.fixType;
  nav.generatedUs = nowUs();
  nav.measurementUs = g.lastValidFixUs;
  nav.sourceBootId = bootId;
  nav.canonicalCrc = boat::canonicalCrc(&nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
  enqueueControl(boat::Type::GnssNavV2, &nav, sizeof(nav));
}

void sendHeartbeat() {
  boat::HeartbeatPayload heartbeat{millis(), txSequence, 0, 0, 0};
  enqueueControl(boat::Type::Heartbeat, &heartbeat, sizeof(heartbeat));
}

void sendTimeSync() {
  boat::TimeSyncRequestPayload request{++syncSequence, nowUs()};
  enqueueControl(boat::Type::TimeSyncRequest, &request, sizeof(request));
}

uint32_t nextRequestId() { return ++requestSequence; }
uint32_t nextCommandSequence() { return ++structuredCommandSequence; }

void sendMode(boat::ControlMode mode) {
  selectedMode = mode;
  boat::ControlModeCommandPayload command{};
  command.protocolVersion = kControlProtocolVersion;
  command.mode = static_cast<uint8_t>(mode);
  command.requestId = nextRequestId();
  command.commandSequence = nextCommandSequence();
  command.sourceUs = nowUs();
  command.canonicalCrc = boat::canonicalCrc(&command, offsetof(boat::ControlModeCommandPayload, canonicalCrc));
  enqueueControl(boat::Type::ControlModeCommand, &command, sizeof(command));
}

void sendManual() {
  if (!manualConfig.configured) return;
  boat::ManualCommandPayload command{};
  command.protocolVersion = kControlProtocolVersion;
  command.enabledMask = manualConfig.enabledMask;
  command.requestId = nextRequestId();
  command.commandSequence = nextCommandSequence();
  command.sourceUs = nowUs();
  command.leftFrontWing = manualConfig.left;
  command.rightFrontWing = manualConfig.right;
  command.rearYaw = manualConfig.rear;
  command.propulsion = manualConfig.propulsion;
  command.canonicalCrc = boat::canonicalCrc(&command, offsetof(boat::ManualCommandPayload, canonicalCrc));
  enqueueControl(boat::Type::ManualCommand, &command, sizeof(command));
  manualConfig.lastSentUs = nowUs();
}

void sendHeading(float degrees) {
  boat::HeadingTargetPayload command{};
  command.protocolVersion = kControlProtocolVersion;
  command.requestId = nextRequestId();
  command.commandSequence = nextCommandSequence();
  command.sourceUs = nowUs();
  command.targetYawRad = degrees * PI / 180.0f;
  command.canonicalCrc = boat::canonicalCrc(&command, offsetof(boat::HeadingTargetPayload, canonicalCrc));
  enqueueControl(boat::Type::HeadingTarget, &command, sizeof(command));
}

boat::ControlMode parseMode(const String& value) {
  if (value == "attitude") return boat::ControlMode::AttitudeAssist;
  if (value == "heading") return boat::ControlMode::HeadingHold;
  if (value == "auto") return boat::ControlMode::AutoWaypoint;
  return boat::ControlMode::Manual;
}

void updateManualFromWeb() {
  manualConfig.enabledMask = static_cast<uint8_t>(web.arg("mask").toInt()) & boat::ManualAll;
  manualConfig.left = clampf(web.arg("left").toFloat(), -1, 1);
  manualConfig.right = clampf(web.arg("right").toFloat(), -1, 1);
  manualConfig.rear = clampf(web.arg("rear").toFloat(), -1, 1);
  manualConfig.propulsion = clampf(web.arg("propulsion").toFloat(), -0.60f, 0.60f);
  manualConfig.configured = manualConfig.enabledMask != 0;
}

bool parseWaypoints(const String& text, boat::WaypointSetPayload& payload) {
  String normalized = text;
  normalized.replace("\r", "");
  normalized.replace("\n", ";");
  uint8_t count = 0;
  int start = 0;
  while (start < normalized.length() && count < boat::kMaxWaypoints) {
    int end = normalized.indexOf(';', start);
    if (end < 0) end = normalized.length();
    String point = normalized.substring(start, end);
    point.trim();
    if (point.length()) {
      const int comma = point.indexOf(',');
      if (comma <= 0) return false;
      const double lat = point.substring(0, comma).toDouble();
      const double lon = point.substring(comma + 1).toDouble();
      if (!isfinite(lat) || !isfinite(lon) || lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;
      payload.points[count++] = {lat, lon};
    }
    start = end + 1;
  }
  payload.count = count;
  return count > 0;
}

