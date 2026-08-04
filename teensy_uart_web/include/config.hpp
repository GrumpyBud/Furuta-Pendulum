#pragma once

#include <cstdint>

#include "control_math.hpp"

namespace config {

// Teensy 4.1 hardware. Serial1 is pin 0 RX and pin 1 TX. The AS5048A is on
// the default SPI bus: CS 10, MOSI 11, MISO 12, SCK 13.
constexpr uint8_t kPendulumChipSelectPin = 10;
constexpr uint8_t kEstopPin = 6;  // normally-closed switch pulls pin LOW
constexpr uint32_t kEncoderSpiHz = 1000000;
constexpr uint32_t kODriveUartBaud = 115200;
constexpr uint8_t kODriveAxis = 0;
constexpr float kMotorTurnsToArmRadians = furuta::kTwoPi;
constexpr float kMotorDirection = 1.0F;
constexpr float kPendulumDirection = 1.0F;

// UART ASCII needs request/response time. 200 Hz is an intentional compromise
// at 115200 baud; raise the baud before raising this rate.
constexpr uint32_t kControlPeriodUs = 5000;  // 200 Hz
constexpr uint32_t kUartResponseTimeoutUs = 4500;
constexpr uint32_t kFeedbackTimeoutUs = 15000;
constexpr uint32_t kODriveHealthPollMs = 200;
constexpr uint32_t kODriveInactiveHealthPollMs = 500;
constexpr uint32_t kTelemetryPeriodMs = 40;  // 25 Hz over USB
constexpr uint32_t kEncoderDiagnosticsPeriodMs = 100;
constexpr float kVelocityFilterHz = 35.0F;

// Safety limits. Start below these values during first mechanical tests.
constexpr float kTorqueLimitNm = 0.75F;
constexpr float kSwingTorqueLimitNm = 0.35F;
constexpr float kTuningTorqueLimitNm = 0.20F;
constexpr float kTorqueSlewNmPerSecond = 8.0F;
constexpr float kArmAngleLimitRad = 2.4F;
constexpr float kArmVelocityLimitRadS = 18.0F;
constexpr float kPendulumVelocityLimitRadS = 50.0F;
constexpr float kZeroMaximumArmRateRadS = 0.20F;
constexpr float kZeroMaximumPendulumRateRadS = 0.35F;
constexpr uint32_t kMaximumConsecutiveEncoderErrors = 3;
constexpr uint32_t kMaximumConsecutiveDeadlineMisses = 2;
constexpr char kODriveWatchdogSeconds[] = "0.05";

// Tuning is balance-only. The user must hold the web control continuously.
constexpr float kTuningStartAngleRad = 0.14F;
constexpr float kTuningAbortAngleRad = 0.32F;
constexpr float kTuningStartRateRadS = 1.0F;
constexpr uint32_t kTuningKeepaliveTimeoutMs = 450;
constexpr uint32_t kTuningMaximumRunMs = 8000;
constexpr uint32_t kRunKeepaliveTimeoutMs = 750;

// Measured plant values belong here, not in the web page.
constexpr float kPendulumMassKg = 0.125F;
constexpr float kPendulumComLengthM = 0.145F;
constexpr float kPendulumInertiaKgM2 = 0.0035F;
constexpr float kSwingEnergyGain = 7.0F;
constexpr float kSwingArmDamping = 0.08F;
constexpr float kSwingArmCentering = 0.10F;  // Nm per arm radian
constexpr float kSwingStartupKickNm = 0.10F;
constexpr uint32_t kSwingStartupKickPhaseMs = 180;
constexpr float kCatchAngleRad = 0.22F;
constexpr float kDropAngleRad = 0.45F;
constexpr float kCatchPendulumVelocityRadS = 3.0F;

// Starting values only. These are not universal gains.
constexpr furuta::Gains kDefaultBalanceGains{1.8F, -18.0F, 1.25F, -2.2F};
constexpr furuta::Gains kGainAbsoluteLimits{8.0F, 60.0F, 8.0F, 12.0F};

static_assert(kMotorDirection == 1.0F || kMotorDirection == -1.0F,
              "kMotorDirection must be exactly 1 or -1");
static_assert(kPendulumDirection == 1.0F || kPendulumDirection == -1.0F,
              "kPendulumDirection must be exactly 1 or -1");
static_assert(kUartResponseTimeoutUs < kControlPeriodUs,
              "UART response timeout must be shorter than the loop period");

}  // namespace config
