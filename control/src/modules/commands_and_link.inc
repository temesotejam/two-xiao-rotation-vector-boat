void sendAck(uint32_t requestId, uint32_t commandSequence, boat::Type type, uint8_t disposition, uint16_t reason) {
  boat::CommandAckPayload ack{};
  ack.requestId = requestId;
  ack.commandSequence = commandSequence;
  ack.commandType = static_cast<uint8_t>(type);
  ack.disposition = disposition;
  ack.safetyState = static_cast<uint8_t>(safetyState);
  ack.controlMode = static_cast<uint8_t>(controlMode);
  ack.reason = reason;
  ack.receivedUs = nowUs();
  ack.appliedUs = nowUs();
  ack.canonicalCrc = boat::canonicalCrc(&ack, offsetof(boat::CommandAckPayload, canonicalCrc));
  enqueue(boat::Type::CommandAck, &ack, sizeof(ack));
}

bool structuredSequenceAccepted(uint32_t sequence) {
  if (sequence == 0 || sequence <= lastStructuredCommandSequence) return false;
  lastStructuredCommandSequence = sequence;
  return true;
}

bool armConditions(uint16_t& reason) {
  const uint64_t t = nowUs();
  if (!linkFresh(t)) { reason = static_cast<uint16_t>(boat::StopReason::LinkTimeout); return false; }
  if (!pcaReady) { reason = static_cast<uint16_t>(boat::StopReason::PcaFailure); return false; }
  if (controlMode == boat::ControlMode::Manual) {
    if (!manualState.enabledMask || t - manualState.receivedUs > kManualCommandTimeoutUs) { reason = static_cast<uint16_t>(boat::StopReason::ManualTimeout); return false; }
    if ((manualState.enabledMask & boat::ManualPropulsion) && !propulsionHealthy(t)) { reason = static_cast<uint16_t>(boat::StopReason::VescInvalid); return false; }
    return true;
  }
  if (!imuFresh(t)) { reason = static_cast<uint16_t>(boat::StopReason::ImuInvalid); return false; }
  if (controlMode == boat::ControlMode::AutoWaypoint) {
    if (!gnssFresh(t) || waypointCount == 0) { reason = static_cast<uint16_t>(boat::StopReason::GnssInvalid); return false; }
    if (!propulsionHealthy(t)) { reason = static_cast<uint16_t>(boat::StopReason::VescInvalid); return false; }
  }
  return true;
}

void handleSimpleCommand(const boat::Frame& frame, boat::Type type) {
  const uint32_t id = frame.header.sequence;
  if (id == 0 || id <= lastSimpleCommandFrameSequence) { sendAck(id, id, type, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand)); return; }
  lastSimpleCommandFrameSequence = id;
  if (type == boat::Type::Estop) {
    transitionTo(boat::SafetyState::EStop, boat::StopReason::Estop);
    sendAck(id, id, type, 0, 0);
    return;
  }
  if (type == boat::Type::ClearEstop) {
    if (safetyState != boat::SafetyState::EStop) { sendAck(id, id, type, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand)); return; }
    transitionTo(boat::SafetyState::Disarmed, boat::StopReason::None);
    sendAck(id, id, type, 0, 0);
    return;
  }
  if (safetyState == boat::SafetyState::EStop) { sendAck(id, id, type, 1, static_cast<uint16_t>(boat::StopReason::Estop)); return; }
  if (type == boat::Type::Stop) {
    if (safetyState != boat::SafetyState::EStop) transitionTo(boat::SafetyState::Disarmed, boat::StopReason::UserStop);
    sendAck(id, id, type, 0, 0);
    return;
  }
  if (type == boat::Type::Disarm) {
    transitionTo(boat::SafetyState::Disarmed, boat::StopReason::None);
    sendAck(id, id, type, 0, 0);
    return;
  }
  if (type == boat::Type::Arm) {
    if (safetyState != boat::SafetyState::Disarmed) { sendAck(id, id, type, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand)); return; }
    uint16_t reason = 0;
    if (!armConditions(reason)) { sendAck(id, id, type, 1, reason); return; }
    transitionTo(boat::SafetyState::ArmedIdle, boat::StopReason::None);
    sendAck(id, id, type, 0, 0);
    return;
  }
  if (type == boat::Type::Start) {
    if (safetyState != boat::SafetyState::ArmedIdle) { sendAck(id, id, type, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand)); return; }
    uint16_t reason = 0;
    if (!armConditions(reason)) { sendAck(id, id, type, 1, reason); return; }
    transitionTo(boat::SafetyState::Running, boat::StopReason::None);
    sendAck(id, id, type, 0, 0);
    return;
  }
  sendAck(id, id, type, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand));
}

void handleModeCommand(const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::ControlModeCommandPayload)) return;
  boat::ControlModeCommandPayload command{};
  memcpy(&command, frame.payload, sizeof(command));
  const uint32_t crc = boat::canonicalCrc(&command, offsetof(boat::ControlModeCommandPayload, canonicalCrc));
  if (crc != command.canonicalCrc || command.protocolVersion != kControlProtocolVersion || !structuredSequenceAccepted(command.commandSequence)) {
    sendAck(command.requestId, command.commandSequence, boat::Type::ControlModeCommand, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand));
    return;
  }
  if (safetyState != boat::SafetyState::Disarmed || command.mode > static_cast<uint8_t>(boat::ControlMode::AutoWaypoint)) {
    sendAck(command.requestId, command.commandSequence, boat::Type::ControlModeCommand, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand));
    return;
  }
  controlMode = static_cast<boat::ControlMode>(command.mode);
  sendAck(command.requestId, command.commandSequence, boat::Type::ControlModeCommand, 0, 0);
}

void handleManualCommand(const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::ManualCommandPayload)) return;
  boat::ManualCommandPayload command{};
  memcpy(&command, frame.payload, sizeof(command));
  const uint32_t crc = boat::canonicalCrc(&command, offsetof(boat::ManualCommandPayload, canonicalCrc));
  if (crc != command.canonicalCrc || command.protocolVersion != kControlProtocolVersion || !structuredSequenceAccepted(command.commandSequence)) {
    sendAck(command.requestId, command.commandSequence, boat::Type::ManualCommand, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand));
    return;
  }
  if (safetyState == boat::SafetyState::EStop || safetyState == boat::SafetyState::Fault) {
    sendAck(command.requestId, command.commandSequence, boat::Type::ManualCommand, 1, static_cast<uint16_t>(stopReason));
    return;
  }
  manualState.enabledMask = command.enabledMask & boat::ManualAll;
  manualState.left = clampf(command.leftFrontWing, -1, 1);
  manualState.right = clampf(command.rightFrontWing, -1, 1);
  manualState.rear = clampf(command.rearYaw, -1, 1);
  manualState.propulsion = clampf(command.propulsion, -kVescMaxDuty, kVescMaxDuty);
  manualState.commandSequence = command.commandSequence;
  manualState.receivedUs = nowUs();
  sendAck(command.requestId, command.commandSequence, boat::Type::ManualCommand, 0, 0);
}

void handleHeadingCommand(const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::HeadingTargetPayload)) return;
  boat::HeadingTargetPayload command{};
  memcpy(&command, frame.payload, sizeof(command));
  const uint32_t crc = boat::canonicalCrc(&command, offsetof(boat::HeadingTargetPayload, canonicalCrc));
  if (crc != command.canonicalCrc || command.protocolVersion != kControlProtocolVersion || !isfinite(command.targetYawRad) || !structuredSequenceAccepted(command.commandSequence)) {
    sendAck(command.requestId, command.commandSequence, boat::Type::HeadingTarget, 1, static_cast<uint16_t>(boat::StopReason::InvalidCommand));
    return;
  }
  headingTargetRad = wrapPi(command.targetYawRad);
  sendAck(command.requestId, command.commandSequence, boat::Type::HeadingTarget, 0, 0);
}

void handleWaypointSet(const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::WaypointSetPayload)) return;
  boat::WaypointSetPayload command{};
  memcpy(&command, frame.payload, sizeof(command));
  boat::WaypointAckPayload ack{};
  ack.requestId = command.requestId;
  ack.revision = waypointRevision;
  const uint32_t crc = boat::canonicalCrc(&command, offsetof(boat::WaypointSetPayload, canonicalCrc));
  if (crc != command.canonicalCrc || safetyState != boat::SafetyState::Disarmed || command.count > boat::kMaxWaypoints) {
    ack.status = 1;
    ack.reason = static_cast<uint8_t>(boat::StopReason::InvalidCommand);
  } else {
    waypointCount = command.count;
    activeWaypoint = 0;
    waypointReachM = clampf(command.reachRadiusM, 0.5f, 20.0f);
    for (uint8_t i = 0; i < waypointCount; ++i) waypoints[i] = command.points[i];
    waypointRevision = command.revision ? command.revision : waypointRevision + 1;
    ack.revision = waypointRevision;
    ack.status = 0;
    navigationState.originSet = false;
  }
  ack.activeIndex = activeWaypoint;
  ack.count = waypointCount;
  ack.canonicalCrc = boat::canonicalCrc(&ack, offsetof(boat::WaypointAckPayload, canonicalCrc));
  enqueue(boat::Type::WaypointAck, &ack, sizeof(ack));
}

void handleGnss(const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::GnssNavV2Payload)) return;
  boat::GnssNavV2Payload nav{};
  memcpy(&nav, frame.payload, sizeof(nav));
  const uint32_t crc = boat::canonicalCrc(&nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
  if (crc != nav.canonicalCrc) return;
  gnssState.valid = (nav.flags & boat::NavFixValid) && (nav.flags & boat::NavLatValid) && (nav.flags & boat::NavLonValid);
  gnssState.latitudeDeg = nav.latitudeE7 / 1e7;
  gnssState.longitudeDeg = nav.longitudeE7 / 1e7;
  gnssState.speedMps = nav.speedMmPerSec / 1000.0f;
  gnssState.courseRad = degToRad(nav.courseE5Deg / 100000.0f);
  gnssState.satellites = static_cast<uint8_t>(nav.satellites > 255 ? 255 : nav.satellites);
  gnssState.measurementUs = nav.measurementUs;
  gnssState.receivedUs = nowUs();
  gnssState.sequence = nav.navSequence;
}

void handleFrame(const boat::Frame& frame) {
  ++rxFrames;
  if (previousRxSequence && frame.header.sequence > previousRxSequence + 1) rxSequenceGaps += frame.header.sequence - previousRxSequence - 1;
  previousRxSequence = frame.header.sequence;
  const boat::Type type = static_cast<boat::Type>(frame.header.type);
  if (type == boat::Type::Heartbeat) { lastHostHeartbeatUs = nowUs(); return; }
  if (type == boat::Type::GnssNavV2) { handleGnss(frame); return; }
  if (type == boat::Type::TimeSyncRequest && frame.header.length == sizeof(boat::TimeSyncRequestPayload)) {
    boat::TimeSyncRequestPayload request{}; memcpy(&request, frame.payload, sizeof(request));
    boat::TimeSyncReplyPayload reply{request.sequence, request.t1Us, frame.uartRxUs, nowUs()};
    enqueue(boat::Type::TimeSyncReply, &reply, sizeof(reply));
    return;
  }
  if (type == boat::Type::TimeSyncEstimate && frame.header.length == sizeof(boat::TimeSyncEstimatePayload)) {
    boat::TimeSyncEstimatePayload estimate{}; memcpy(&estimate, frame.payload, sizeof(estimate));
    controlMinusCommunicationUs = estimate.controlMinusCommunicationUs;
    timeSyncRttUs = estimate.rttUs;
    return;
  }
  if (type == boat::Type::ControlModeCommand) { handleModeCommand(frame); return; }
  if (type == boat::Type::ManualCommand) { handleManualCommand(frame); return; }
  if (type == boat::Type::HeadingTarget) { handleHeadingCommand(frame); return; }
  if (type == boat::Type::WaypointSet) { handleWaypointSet(frame); return; }
  if (type == boat::Type::Arm || type == boat::Type::Disarm || type == boat::Type::Start || type == boat::Type::Stop || type == boat::Type::Estop || type == boat::Type::ClearEstop) {
    handleSimpleCommand(frame, type);
  }
}

void serviceLink() {
  for (uint16_t i = 0; i < kLinkRxByteBudget && linkUart.available(); ++i) {
    boat::Frame frame{};
    if (linkDecoder.feed(static_cast<uint8_t>(linkUart.read()), frame)) {
      frame.uartRxUs = nowUs();
      handleFrame(frame);
    }
  }
}

