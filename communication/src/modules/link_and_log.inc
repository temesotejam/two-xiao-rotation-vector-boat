bool enqueueControl(boat::Type type, const void* payload = nullptr, size_t length = 0) {
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

void enqueueLog(const boat::Header& sourceHeader, const uint8_t* payload, uint64_t localUs, bool outbound) {
  if (!logging || !logQueue) return;
  LogItem item{};
  item.localUs = localUs;
  item.header = sourceHeader;
  if (outbound) item.header.flags |= 0x8000u;
  item.length = sourceHeader.length;
  if (item.length && payload) memcpy(item.payload, payload, item.length);
  if (xQueueSend(logQueue, &item, 0) != pdPASS) ++logDrops;
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
  if (!count || controlUart.write(encoded, count) != count) {
    ++txDrops;
    return;
  }
  ++txFrames;
  enqueueLog(header, item.payload, nowUs(), true);
}

void serviceTx() {
  TxItem item{};
  for (uint8_t i = 0; i < 8 && xQueueReceive(txQueue, &item, 0) == pdTRUE; ++i) sendItem(item);
}

String nextLogName() {
  if (!SD.exists(kLogDirectory)) SD.mkdir(kLogDirectory);
  for (uint16_t n = 1; n < 10000; ++n) {
    char path[32]{};
    snprintf(path, sizeof(path), "%s/RUN%04u.BIN", kLogDirectory, n);
    if (!SD.exists(path)) return String(path);
  }
  return String();
}

struct __attribute__((packed)) LogFileHeader {
  char magic[4];
  uint16_t version;
  uint16_t headerBytes;
  uint32_t communicationBootId;
  uint64_t openedUs;
};

struct __attribute__((packed)) LogRecordHeader {
  uint32_t recordBytes;
  uint64_t localUs;
  boat::Header frame;
};

void logTask(void*) {
  File file;
  uint32_t unflushed = 0;
  uint64_t lastFlushUs = 0;
  for (;;) {
    if (logCommand == 1) {
      logCommand = 0;
      if (file) file.close();
      xQueueReset(logQueue);
      activeLogName = nextLogName();
      logging = false;
      logRecords = 0;
      if (sdReady && activeLogName.length()) {
        file = SD.open(activeLogName, FILE_WRITE);
        if (file) {
          const LogFileHeader header{{'B','L','G','2'}, 2, sizeof(LogFileHeader), bootId, nowUs()};
          logging = file.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header)) == sizeof(header);
          lastFlushUs = nowUs();
          unflushed = 0;
        }
      }
    }
    if (logCommand == 2) {
      logCommand = 0;
      logging = false;
      LogItem discard{};
      while (xQueueReceive(logQueue, &discard, 0) == pdTRUE) {}
      if (file) { file.flush(); file.close(); }
    }
    LogItem item{};
    if (logging && xQueueReceive(logQueue, &item, pdMS_TO_TICKS(kLogTaskWakeMs)) == pdTRUE) {
      LogRecordHeader record{};
      record.recordBytes = sizeof(record) + item.length + sizeof(uint32_t);
      record.localUs = item.localUs;
      record.frame = item.header;
      uint32_t crc = boat::crc32(reinterpret_cast<const uint8_t*>(&record), sizeof(record));
      if (item.length) crc ^= boat::crc32(item.payload, item.length);
      const size_t a = file.write(reinterpret_cast<const uint8_t*>(&record), sizeof(record));
      const size_t b = item.length ? file.write(item.payload, item.length) : 0;
      const size_t c = file.write(reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));
      if (a != sizeof(record) || (item.length && b != item.length) || c != sizeof(crc)) {
        ++logDrops;
        logging = false;
        file.close();
      } else {
        ++logRecords;
        ++unflushed;
      }
    }
    const uint64_t t = nowUs();
    if (logging && file && (unflushed >= 64 || t - lastFlushUs >= 1000000ULL)) {
      file.flush();
      unflushed = 0;
      lastFlushUs = t;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

