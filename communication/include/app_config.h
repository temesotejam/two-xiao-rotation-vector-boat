#pragma once
#include <Arduino.h>
namespace app_config {
constexpr char kFirmwareName[] = "two-xiao-boat-communication";
constexpr char kFirmwareVersion[] = "1.0.0-production";
constexpr char kApSsid[] = "BOAT-CONTROL";
constexpr char kApPassword[] = "12345678";
constexpr uint16_t kHttpPort = 80;
constexpr int kGnssRxPin = D0;
constexpr int kGnssTxPin = D1;
constexpr int kControlUartRxPin = D7;
constexpr int kControlUartTxPin = D6;
constexpr int kSdCsPin = 21;
constexpr int kSdSckPin = D8;
constexpr int kSdMisoPin = D9;
constexpr int kSdMosiPin = D10;
constexpr uint32_t kGnssBaud = 115200UL;
constexpr uint32_t kControlUartBaud = 921600UL;
constexpr uint16_t kGnssUartRxBufferBytes = 2048;
constexpr uint16_t kGnssReadBudgetBytes = 512;
constexpr uint16_t kGnssInputLineChars = 127;
constexpr uint16_t kGnssMaxSentenceChars = 110;
constexpr uint32_t kGnssSentenceTimeoutMs = 500UL;
constexpr uint32_t kGnssNoDataTimeoutMs = 3000UL;
constexpr uint32_t kGnssNavIntervalMs = 100UL;
constexpr uint32_t kHeartbeatIntervalMs = 100UL;
constexpr uint32_t kTimeSyncIntervalMs = 1000UL;
constexpr uint32_t kManualRefreshIntervalMs = 200UL;
constexpr uint32_t kDiagnosticIntervalMs = 1000UL;
constexpr uint32_t kLogTaskWakeMs = 5UL;
constexpr UBaseType_t kLogTaskPriority = 2;
constexpr char kLogDirectory[] = "/BOATLOG";
constexpr uint8_t kControlProtocolVersion = 1;
constexpr uint8_t kTxQueueDepth = 24;
constexpr uint8_t kLogQueueDepth = 24;
}
