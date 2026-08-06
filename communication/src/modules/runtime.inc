void setup() {
  Serial.begin(115200);
  delay(100);
  bootId = esp_random();
  txQueue = xQueueCreate(kTxQueueDepth, sizeof(TxItem));
  logQueue = xQueueCreate(kLogQueueDepth, sizeof(LogItem));
  controlUart.begin(kControlUartBaud, SERIAL_8N1, kControlUartRxPin, kControlUartTxPin);
  gnssUart.setRxBufferSize(kGnssUartRxBufferBytes);
  gnssUart.begin(kGnssBaud, SERIAL_8N1, kGnssRxPin, kGnssTxPin);
  gnssRx.begin(gnssUart);
  SPI.begin(kSdSckPin, kSdMisoPin, kSdMosiPin, kSdCsPin);
  sdReady = SD.begin(kSdCsPin, SPI);
  xTaskCreatePinnedToCore(logTask, "SdLog", 6144, nullptr, kLogTaskPriority, nullptr, 0);
  beginWeb();
  Serial.printf("%s %s boot=%lu SD=%d AP=%s IP=%s\n", kFirmwareName, kFirmwareVersion, (unsigned long)bootId,
                sdReady, kApSsid, WiFi.softAPIP().toString().c_str());
}

void loop() {
  serviceGnss();
  serviceControlRx();
  serviceTx();
  web.handleClient();
  const uint64_t t = nowUs();
  if (t - lastGnssNavUs >= static_cast<uint64_t>(kGnssNavIntervalMs) * 1000ULL) { lastGnssNavUs = t; sendGnssNav(); }
  if (t - lastHeartbeatUs >= static_cast<uint64_t>(kHeartbeatIntervalMs) * 1000ULL) { lastHeartbeatUs = t; sendHeartbeat(); }
  if (t - lastSyncUs >= static_cast<uint64_t>(kTimeSyncIntervalMs) * 1000ULL) { lastSyncUs = t; sendTimeSync(); }
  if (manualConfig.configured && t - manualConfig.lastSentUs >= static_cast<uint64_t>(kManualRefreshIntervalMs) * 1000ULL) sendManual();
  if (t - lastDiagnosticUs >= static_cast<uint64_t>(kDiagnosticIntervalMs) * 1000ULL) {
    lastDiagnosticUs = t;
    Serial.printf("COMM link_age_ms=%lu rx=%lu tx=%lu gaps=%lu crc=%lu gnss=%d sd=%d log=%d records=%lu rtt_us=%lu\n",
      (unsigned long)(lastControlRxUs ? (t - lastControlRxUs) / 1000ULL : UINT32_MAX), (unsigned long)rxFrames,
      (unsigned long)txFrames, (unsigned long)rxSequenceGaps, (unsigned long)controlDecoder.crcErrors,
      gnssRx.receiving(t), sdReady, logging, (unsigned long)logRecords, (unsigned long)syncRttUs);
  }
  delay(1);
}
