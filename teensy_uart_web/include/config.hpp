#pragma once

#include <cstdint>

#include "control_math.hpp"

namespace config {

// Teensy 4.1 hardware. Serial1 is pin 0 RX and pin 1 TX. The AS5048A is on
// the default SPI bus: CS 10, MOSI 11, MISO 12, SCK 13.
constexpr uint8_t kPendulumChipSelectPin = 10;
constexpr uint32_t kEncoderSpiHz = 1000000;
constexpr uint32_t kODriveUartBaud = 115200;
constexpr uint8_t kODriveAxis = 0;
// Measured transmission: direct drive, one motor revolution per arm revolution.
constexpr float kMotorRevolutionsPerArmRevolution = 1.0F;
constexpr float kMotorTurnsToArmRadians =
    furuta::kTwoPi / kMotorRevolutionsPerArmRevolution;
// Recorded ODrive/power hardware values. These do not program the ODrive.
// Confirmed conservative motor-current limits for commissioning.
constexpr float kODriveConfiguredCurrentSoftMaxAmp = 10.0F;
constexpr float kODriveConfiguredCurrentHardMaxAmp = 18.0F;
constexpr float kODriveConfiguredVelocityLimitTurnsPerSecond = 1.5F;
constexpr bool kODriveTorqueModeVelocityLimitEnabled = true;
constexpr float kODriveTorqueModeVelocityGain = 0.167F;
constexpr float kMotorTorqueConstantNmPerAmp = 0.0306296F;  // D5065 270 KV
constexpr bool kODriveDcPositiveCurrentTripEnabled = true;
constexpr bool kODriveDcNegativeCurrentTripEnabled = true;
constexpr float kODriveDcMaxPositiveCurrentAmp = 25.0F;
constexpr float kODriveDcMaxNegativeCurrentAmp = -5.0F;
constexpr float kODriveUnderVoltageTripV = 19.8F;
constexpr float kODriveOverVoltageTripV = 25.5F;
constexpr float kDcBreakerCurrentAmp = 30.0F;
constexpr uint8_t kBatterySeriesCells = 6;
constexpr float kBatteryCapacityAh = 5.0F;
constexpr float kBatteryDischargeRatingC = 100.0F;
constexpr float kBatteryNominalVoltageV = 22.2F;
constexpr float kBatteryFullChargeVoltageV = 25.2F;
constexpr float kBrakeResistorOhm = 2.0F;
constexpr float kBrakeResistorContinuousPowerW = 50.0F;
constexpr bool kODriveBrakeResistorEnabled = true;
// Global ODrive setting: 0 A sends all estimated regenerative current to the
// brake resistor instead of intentionally returning it to the battery.
constexpr float kODriveMaxRegenCurrentAmp = 0.0F;
constexpr bool kODriveDcBusVoltageFeedbackEnabled = false;
// Recorded for diagnostics; these exceed the 25.5 V trip and are unsuitable
// for this 6S bus. They are inactive while voltage feedback remains disabled.
constexpr float kODriveVoltageFeedbackRampStartV = 51.0F;
constexpr float kODriveVoltageFeedbackRampEndV = 53.0F;
constexpr float kMotorDirection = 1.0F;
// Hardware sign check: whole-arm clockwise motion is logical-positive. The
// pendulum encoder count decreases for clockwise pendulum motion, while the
// observed inertial coupling makes arm CW -> pendulum CW (and CCW -> CCW).
// Negating the raw pendulum coordinate therefore makes positive arm
// acceleration produce positive pendulum acceleration as assumed by the model.
constexpr float kPendulumDirection = -1.0F;

// UART ASCII needs request/response time. 200 Hz is an intentional compromise
// at 115200 baud; raise the baud before raising this rate.
constexpr uint32_t kControlPeriodUs = 5000;  // 200 Hz
constexpr uint32_t kUartResponseTimeoutUs = 4500;
// Long property names can take nearly the entire fast timeout just to leave
// the Teensy TX pin at 115200 baud. This larger timeout is used only during
// disarmed setup/arming verification, never inside the active 200 Hz loop.
constexpr uint32_t kUartConfigurationResponseTimeoutUs = 12000;
constexpr uint32_t kFeedbackTimeoutUs = 15000;
constexpr uint32_t kODriveHealthPollMs = 200;
constexpr uint32_t kODriveInactiveHealthPollMs = 500;
constexpr uint32_t kTelemetryPeriodMs = 40;        // 25 Hz normally
constexpr uint32_t kTuningTelemetryPeriodMs = 10;  // 100 Hz during gain trials
constexpr uint32_t kEncoderDiagnosticsPeriodMs = 100;
// Hardware trials showed a repeatable upright catch followed by rapid torque
// reversals dominated by the pendulum-rate term. 25 Hz keeps useful damping
// while attenuating more AS5048A finite-difference noise than the original
// 35 Hz commissioning value.
constexpr float kVelocityFilterHz = 25.0F;

// Conservative commissioning limits expressed as phase current first, then
// converted to torque with the measured ODrive torque constant. These are
// firmware command clamps, not a substitute for matching ODrive limits.
constexpr float kCommissioningPhaseCurrentLimitAmp = 10.0F;
constexpr float kSwingPhaseCurrentLimitAmp = 6.5F;
constexpr float kSwingTuningPhaseCurrentLimitAmp = 4.0F;
constexpr float kTuningPhaseCurrentLimitAmp = 4.0F;
constexpr float kTorqueLimitNm =
    kCommissioningPhaseCurrentLimitAmp * kMotorTorqueConstantNmPerAmp;
constexpr float kSwingTorqueLimitNm =
    kSwingPhaseCurrentLimitAmp * kMotorTorqueConstantNmPerAmp;
constexpr float kSwingTuningTorqueLimitNm =
    kSwingTuningPhaseCurrentLimitAmp * kMotorTorqueConstantNmPerAmp;
constexpr float kTuningTorqueLimitNm =
    kTuningPhaseCurrentLimitAmp * kMotorTorqueConstantNmPerAmp;
constexpr float kTorqueSlewNmPerSecond = 8.0F;
constexpr float kArmAngleLimitRad = 2.4F;
// Fault early enough to leave coasting distance before the 2.4 rad soft limit.
// This is still not a substitute for physical stops at or before +/- pi.
constexpr float kArmTravelPredictionSeconds = 0.080F;
constexpr float kArmVelocityLimitRadS =
    kODriveConfiguredVelocityLimitTurnsPerSecond * furuta::kTwoPi;
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
// Three repeatable 8 s hands-off trials passed with the hardware-tuned gains
// and 25 Hz velocity filter. The next guarded validation window is 20 s.
constexpr uint32_t kTuningMaximumRunMs = 20000;
constexpr uint32_t kSwingTuningMaximumRunMs = 20000;
// The focused browser sends this only while Space is physically held.
constexpr uint32_t kBrowserDeadmanTimeoutMs = 450;

// Mechanism characterization. The 0.159 kg measured lump includes the rod,
// printed block, horizontal shaft, collars, set screws, and one complete bearing
// as an approximation for the two bearing assemblies. Everything except the rod
// is provisionally placed on the pendulum pivot axis. The corrected assembled
// upright yaw inertia below already allocates this hardware and must not be
// combined with an additional point-mass m*r^2 approximation.
constexpr float kPendulumMassKg = 0.159F;
constexpr float kPendulumSmallOscillationPeriodS = 0.7327F;
constexpr float kPendulumRodLengthM = 0.200F;
constexpr float kPendulumRodDiameterM = 0.008F;
constexpr float kPendulumRodStartOffsetM = 0.00635F;  // 1/4 inch
constexpr float kPendulumRodDensityKgM3 = 8000.0F;    // nominal stainless
constexpr float kPendulumRodMassKg = 0.08042F;
constexpr float kPendulumNearPivotHardwareMassKg = 0.07858F;
constexpr float kPendulumComLengthM = 0.05379F;
constexpr float kPendulumInertiaKgM2 = 0.001141F;  // from COM and measured T
constexpr float kPreviouslyReportedArmPivotRadiusM = 0.3215F;
constexpr float kArmExtrusionLengthM = 0.140F;
constexpr float kArmRigidAssemblyNominalMassKg = 0.1597F;
// The 0.3215 m radius used the CAD's 0.280 m extrusion. Shortening only that
// member to the real 0.140 m gives 0.3215 - 0.140 = 0.1815 m.
constexpr float kArmPivotRadiusM = 0.1815F;
constexpr float kPendulumFirstMomentKgM =
    kPendulumMassKg * kPendulumComLengthM;
constexpr float kArmPendulumCouplingKgM2 =
    kArmPivotRadiusM * kPendulumFirstMomentKgM;
// Generalized yaw inertia M(0,0) with the pendulum upright. Unlike m*r^2,
// this uses each corrected STEP body's real radial position. It includes the
// real 200 mm rod, allowances for collars/encoder/screws, and rotor inertia.
constexpr float kUprightArmAxisInertiaKgM2 = 0.00561F;
constexpr float kUprightArmAxisInertiaMinimumKgM2 = 0.00510F;
constexpr float kUprightArmAxisInertiaMaximumKgM2 = 0.00620F;
constexpr float kArmViscousDampingNmPerRadS = 0.0F;
constexpr float kPendulumViscousDampingNmPerRadS = 0.0F;
constexpr float kTransmissionEfficiency = 1.0F;  // direct-drive starting value

// Set true only after the motor-disabled coupling-sign procedure in the README
// confirms kPendulumDirection. This is the final compile-time tuning interlock.
constexpr bool kControlDirectionVerified = true;
constexpr bool kMechanismSetupComplete = kControlDirectionVerified;
// Upright trials must be repeatably stable and their logs reviewed before this
// separate interlock is enabled. It prevents unvalidated automatic swing-up.
constexpr bool kAutomaticSwingUpEnabled = false;

// Guarded swing-up commissioning is separate from unrestricted automatic run.
// It uses the same 4 A phase-current-equivalent clamp as upright tuning, a
// focused-tab dead-man, centered/down start gates, and a 20 s timeout.
constexpr bool kSwingTuningEnabled = true;
constexpr furuta::SwingSettings kDefaultSwingSettings{
    0.80F, 0.030F, 0.040F, 0.040F};
constexpr furuta::SwingSettings kSwingSettingLimits{
    3.00F, 0.120F, 0.120F, 0.080F};
constexpr uint32_t kSwingStartupKickPhaseMs = 180;
constexpr float kSwingStartDownToleranceRad = 0.18F;
constexpr float kSwingStartPendulumRateRadS = 0.50F;
constexpr float kSwingStartArmAngleRad = 0.35F;
constexpr float kSwingStartArmRateRadS = 0.30F;
constexpr float kCatchAngleRad = 0.14F;
constexpr float kDropAngleRad = 0.45F;
constexpr float kCatchPendulumVelocityRadS = 1.5F;

// Discrete LQR at 200 Hz for x=[arm, pendulum, arm_rate, pendulum_rate], with
// positive pendulum chosen so a positive arm acceleration initially produces a
// positive pendulum acceleration. See BALANCING_REVIEW.md for A/B/Q/R.
constexpr furuta::Gains kModelBalanceGains{-0.09140F, 1.44432F, -0.06921F,
                                            0.13886F};
constexpr float kMinimumRuntimeGainScale = 0.50F;
constexpr float kMaximumRuntimeGainScale = 1.50F;
// Best hardware-validated combination so far: three hands-off trials reached
// the complete 8 s tuning window. Automatic swing-up remains independently
// locked until the longer guarded trials are reviewed.
constexpr furuta::Gains kDefaultBalanceGains{-0.07000F, 1.60000F, -0.06920F,
                                              0.09000F};
// Redundant absolute bounds sit outside the enforced 50-150% same-sign model
// profile envelope and reject arbitrary high feedback.
constexpr furuta::Gains kGainAbsoluteLimits{0.16F, 2.50F, 0.13F, 0.25F};

static_assert(kMotorDirection == 1.0F || kMotorDirection == -1.0F,
              "kMotorDirection must be exactly 1 or -1");
static_assert(kMotorRevolutionsPerArmRevolution > 0.0F,
              "motor-to-arm ratio must be positive");
static_assert(kArmAngleLimitRad < furuta::kPi,
              "software arm limit must remain inside +/-180 degrees");
static_assert(kUprightArmAxisInertiaKgM2 * kPendulumInertiaKgM2 >
                  kArmPendulumCouplingKgM2 * kArmPendulumCouplingKgM2,
              "linearized inertia matrix must be positive definite");
static_assert(!kAutomaticSwingUpEnabled || kMechanismSetupComplete,
              "automatic swing-up requires completed upright setup");
static_assert(kCommissioningPhaseCurrentLimitAmp <
                  kODriveConfiguredCurrentHardMaxAmp,
              "firmware current envelope must stay below ODrive hard max");
static_assert(kTuningPhaseCurrentLimitAmp <= kSwingPhaseCurrentLimitAmp &&
                  kSwingTuningPhaseCurrentLimitAmp <=
                      kSwingPhaseCurrentLimitAmp &&
                  kSwingPhaseCurrentLimitAmp <=
                      kCommissioningPhaseCurrentLimitAmp,
              "current limit tiers must be ordered");
static_assert(kPendulumDirection == 1.0F || kPendulumDirection == -1.0F,
              "kPendulumDirection must be exactly 1 or -1");
static_assert(kUartResponseTimeoutUs < kControlPeriodUs,
              "UART response timeout must be shorter than the loop period");
static_assert(kUartConfigurationResponseTimeoutUs < 50000U,
              "arming verification timeout must stay below the ODrive watchdog");

}  // namespace config
