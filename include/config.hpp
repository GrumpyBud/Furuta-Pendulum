#pragma once

#include "control_math.hpp"

namespace config {

// Hardware. CAN1 is Teensy pins 22 (CTX1) and 23 (CRX1). An external 3.3 V
// CAN transceiver is mandatory; never connect these pins directly to CAN-H/L.
constexpr uint32_t kCanBaud = 250000;
constexpr uint32_t kODriveNodeId = 0;
constexpr uint8_t kPendulumEncoderA = 2;
constexpr uint8_t kPendulumEncoderB = 3;
constexpr uint8_t kEstopPin = 6;  // closed switch pulls low; open wire is unsafe
constexpr int32_t kPendulumCountsPerRevolution = 8192;  // after quadrature
constexpr float kMotorTurnsToArmRadians = furuta::kTwoPi; // direct drive
constexpr float kMotorDirection = 1.0F;
constexpr float kPendulumDirection = 1.0F;

// Timing and safety. The ODrive encoder estimate and heartbeat CAN cyclic rates
// should be configured to at least 500 Hz and 10 Hz respectively.
constexpr uint32_t kControlPeriodUs = 2000;  // 500 Hz
constexpr uint32_t kFeedbackTimeoutUs = 20000;
constexpr uint32_t kHeartbeatTimeoutUs = 300000;
constexpr float kTorqueLimitNm = 1.5F;
constexpr float kArmAngleLimitRad = 2.6F;
constexpr float kArmVelocityLimitRadS = 25.0F;
constexpr float kPendulumVelocityLimitRadS = 60.0F;
constexpr float kVelocityFilterHz = 45.0F;

// Controller model. Measure these rather than trusting a CAD material value.
constexpr float kPendulumMassKg = 0.125F;
constexpr float kPendulumComLengthM = 0.145F;
constexpr float kPendulumInertiaKgM2 = 0.0035F;
constexpr float kSwingEnergyGain = 7.0F;
constexpr float kSwingArmDamping = 0.08F;
constexpr float kSwingTorqueLimitNm = 0.65F;
constexpr float kCatchAngleRad = 0.22F;
constexpr float kDropAngleRad = 0.45F;
constexpr float kCatchPendulumVelocityRadS = 3.0F;

// Starting values only. Recompute these gains from the measured linear model.
// State order: arm angle, pendulum angle (upright=0), arm rate, pendulum rate.
constexpr furuta::Gains kBalanceGains{1.8F, -18.0F, 1.25F, -2.2F};

}  // namespace config

