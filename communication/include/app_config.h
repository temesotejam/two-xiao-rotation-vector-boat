#pragma once
#include <Arduino.h>

namespace app_config {
constexpr char kFirmwareName[] = "xiao-boat-communication";
constexpr char kFirmwareVersion[] = "0.4.0-two-xiao-shadow";
constexpr char kApSsid[] = "BOAT-CONTROL";
constexpr char kApPassword[] = "12345678";
constexpr uint16_t kHttpPort = 80;
constexpr int kGnssRxPin = D0, kGnssTxPin = D1;
constexpr int kControlUartRxPin = D7, kControlUartTxPin = D6;
constexpr int kSdCsPin = 21, kSdSckPin = D8, kSdMisoPin = D9, kSdMosiPin = D10;
constexpr uint32_t kGnssBaud = 115200UL, kControlUartBaud = 921600UL;
constexpr uint16_t kGnssUartRxBufferBytes = 2048, kGnssReadBudgetBytes = 512;
constexpr uint16_t kGnssInputLineChars = 127, kGnssMaxSentenceChars = 110;
constexpr uint32_t kGnssSentenceTimeoutMs = 500UL, kGnssNoDataTimeoutMs = 3000UL;
constexpr uint32_t kGnssNavIntervalMs = 100UL, kControlHeartbeatIntervalMs = 100UL, kDiagnosticIntervalMs = 1000UL;
constexpr uint32_t kLogTaskWakeMs = 5UL;
constexpr UBaseType_t kLogTaskPriority = 2;
constexpr char kLogDirectory[] = "/BOATLOG";
}