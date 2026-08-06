#pragma once
#include <Arduino.h>

#ifndef BOAT_DRY_RUN
#define BOAT_DRY_RUN 0
#endif

namespace app_config {
constexpr char kFirmwareName[] = "two-xiao-boat-control";
constexpr char kFirmwareVersion[] = "1.0.0-production";
constexpr bool kDryRun = BOAT_DRY_RUN != 0;

constexpr int kPeripheralSdaPin = D1;
constexpr int kPeripheralSclPin = D0;
constexpr int kBnoRstPin = D2;
constexpr int kBnoIntPin = D3;
constexpr int kBnoSdaPin = D4;
constexpr int kBnoSclPin = D5;
constexpr int kLinkRxPin = D6;
constexpr int kLinkTxPin = D7;
constexpr int kVescRxPin = D8;
constexpr int kVescTxPin = D9;
constexpr int kMotorRelayPin = D10;
constexpr bool kMotorRelayActiveHigh = true;

constexpr uint8_t kBnoAddress = 0x4A;
constexpr uint8_t kBnoAlternateAddress = 0x4B;
constexpr uint8_t kTofAddress = 0x29;
constexpr uint8_t kInaAddress = 0x44;
constexpr uint8_t kPcaAddress = 0x40;
constexpr uint32_t kBnoI2cHz = 100000UL;
constexpr uint32_t kPeripheralI2cHz = 400000UL;
constexpr uint32_t kAccelGyroIntervalUs = 5000UL;
constexpr uint32_t kRotationIntervalUs = 20000UL;
constexpr uint32_t kMagneticIntervalUs = 50000UL;
constexpr uint16_t kBnoEventQueueDepth = 96;
constexpr uint8_t kBnoServiceCallBudget = 8;
constexpr uint32_t kBnoTaskFallbackMs = 2UL;
constexpr uint32_t kBnoNoDataTimeoutMs = 3000UL;
constexpr uint32_t kReinitIntervalMs = 2000UL;

constexpr uint32_t kLinkBaud = 921600UL;
constexpr uint32_t kLinkHeartbeatTimeoutMs = 500UL;
constexpr uint16_t kLinkRxByteBudget = 768;
constexpr uint8_t kLinkTxQueueDepth = 64;

constexpr uint32_t kVescUartBaud = 115200UL;
constexpr uint32_t kVescFrameTimeoutMs = 100UL;
constexpr uint32_t kVescMaxPayloadBytes = 512UL;
constexpr uint32_t kVescRequestIntervalMs = 20UL;
constexpr uint32_t kVescCommandIntervalMs = 50UL;
constexpr uint32_t kVescStaleUs = 300000UL;
constexpr float kVescMaxDuty = 0.60f;
constexpr float kVescRisePerSecond = 0.30f;
constexpr float kVescFallPerSecond = 1.50f;

constexpr uint32_t kOscillatorHz = 25000000UL;
constexpr float kServoPwmHz = 50.0f;
constexpr uint8_t kLeftServoChannel = 0;
constexpr uint8_t kRightServoChannel = 1;
constexpr uint8_t kRearServoChannel = 2;
constexpr uint16_t kServoMinUs = 1200;
constexpr uint16_t kServoNeutralUs = 1500;
constexpr uint16_t kServoMaxUs = 1800;
constexpr float kServoRateUsPerSecond = 300.0f;
constexpr bool kLeftServoReversed = false;
constexpr bool kRightServoReversed = false;
constexpr bool kRearServoReversed = false;

constexpr uint32_t kControlIntervalUs = 20000UL;
constexpr uint32_t kInaSampleIntervalUs = 20000UL;
constexpr uint32_t kTofFrequencyHz = 10UL;
constexpr uint32_t kTelemetryFastIntervalMs = 50UL;
constexpr uint32_t kTelemetrySlowIntervalMs = 100UL;
constexpr uint32_t kDiagnosticIntervalMs = 1000UL;

constexpr uint16_t kInaConfig = 0x08DF;
constexpr uint16_t kInaCalibration = 0x0800;
constexpr uint16_t kConfig = kInaConfig;
constexpr uint16_t kCalibration = kInaCalibration;
constexpr float kShuntOhm = 0.002f;
constexpr float kCurrentLsbA = 0.00125f;
constexpr float kPowerLsbW = 0.03125f;

constexpr uint32_t kImuAttitudeStaleUs = 120000UL;
constexpr uint32_t kImuGyroStaleUs = 60000UL;
constexpr uint32_t kGnssStaleUs = 500000UL;
constexpr uint32_t kTofStaleUs = 250000UL;
constexpr uint32_t kPowerStaleUs = 250000UL;
constexpr uint32_t kManualCommandTimeoutUs = 500000UL;
constexpr float kCriticalPitchRad = 0.70f;
constexpr float kCriticalRollRad = 0.70f;

constexpr float kLowVoltageLimitV = 9.5f;
constexpr float kCriticalVoltageV = 8.5f;
constexpr float kOverCurrentLimitA = 22.0f;
constexpr float kCriticalCurrentA = 28.0f;
constexpr float kStallDuty = 0.25f;
constexpr float kStallCurrentA = 8.0f;
constexpr float kStallErpm = 100.0f;
constexpr uint32_t kStallTripUs = 1000000UL;

constexpr float kTargetHeightM = 0.45f;
constexpr float kKpHeight = 0.75f;
constexpr float kKpPitch = 0.80f;
constexpr float kKdPitch = 0.10f;
constexpr float kKpRoll = 1.25f;
constexpr float kKdRoll = 0.22f;
constexpr float kKpYaw = 0.90f;
constexpr float kKdYaw = 0.12f;
constexpr float kAutoPropulsion = 0.55f;
constexpr float kWaypointReachM = 1.5f;
constexpr float kLosLookaheadM = 4.0f;

constexpr uint8_t kControlProtocolVersion = 1;
}

namespace cfg = app_config;
