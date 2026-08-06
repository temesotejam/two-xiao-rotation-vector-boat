#pragma once
#include <Arduino.h>

namespace app_config {
constexpr bool kActuatorOutputCompileEnable = false;
constexpr int kPeripheralSdaPin = D1, kPeripheralSclPin = D0;
constexpr int kBnoRstPin = D2, kBnoIntPin = D3, kBnoSdaPin = D4, kBnoSclPin = D5;
constexpr int kLinkRxPin = D6, kLinkTxPin = D7, kVescRxPin = D8, kVescTxPin = D9;
constexpr uint8_t kBnoAddress = 0x4A, kBnoAlternateAddress = 0x4B;
constexpr uint8_t kTofAddress = 0x29, kInaAddress = 0x44, kPcaAddress = 0x40;
constexpr uint32_t kBnoI2cHz = 100000UL, kPeripheralI2cHz = 400000UL;
constexpr uint32_t kAccelGyroIntervalUs = 20000UL, kRotationIntervalUs = 20000UL, kMagneticIntervalUs = 50000UL;
constexpr uint16_t kBnoEventQueueDepth = 96;
constexpr uint8_t kBnoServiceCallBudget = 8;
constexpr uint32_t kBnoTaskFallbackMs = 2UL, kBnoNoDataTimeoutMs = 3000UL, kReinitIntervalMs = 2000UL;
constexpr uint32_t kLinkBaud = 921600UL, kLinkFailSafeTimeoutMs = 500UL;
constexpr uint32_t kVescUartBaud = 115200UL, kVescFrameTimeoutMs = 100UL, kVescMaxPayloadBytes = 512UL;
constexpr uint32_t kOscillatorHz = 25000000UL;
constexpr float kServoPwmHz = 50.0f;
constexpr uint16_t kInaCalibration = 0x0800;
constexpr uint16_t kCalibration = kInaCalibration;
constexpr float kShuntOhm = 0.002f, kCurrentLsbA = 0.00125f, kPowerLsbW = 0.03125f;
}

namespace cfg = app_config;
