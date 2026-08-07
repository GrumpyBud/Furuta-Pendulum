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
constexpr float kODriveConfiguredCurrentSoftMaxAmp = 15.0F;
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
constexpr uint32_t kMaximumConsecutiveFeedbackErrors = 2;
// CENTERING and SETTLING run on ODrive's own position loop, so they do not
// need to crowd the UART with 200 feedback requests per second. Allow the S1
// more response time while that loop changes modes, without weakening the
// 200 Hz feedback requirements used by Teensy torque control.
constexpr uint32_t kPositionFeedbackPeriodMs = 10;
constexpr uint32_t kPositionFeedbackResponseTimeoutUs = 9000;
constexpr uint32_t kPositionFeedbackTimeoutUs = 35000;
constexpr uint32_t kPositionMaximumConsecutiveFeedbackErrors = 3;
constexpr uint32_t kODriveHealthPollMs = 200;
constexpr uint32_t kODriveHealthRetryMs = 10;
constexpr uint32_t kMaximumConsecutiveHealthQueryErrors = 3;
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
constexpr float kSwingPhaseCurrentLimitAmp = 15.0F;
constexpr float kSwingTuningPhaseCurrentLimitAmp = 15.0F;
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
// It remains clamped by the configured 10 A ODrive soft limit, a focused-tab
// dead-man, automatic speed-limited preparation, and a 20 s energy timeout.
constexpr bool kSwingTuningEnabled = true;
constexpr furuta::SwingSettings kDefaultSwingSettings{
    0.80F, 0.010F, 0.040F, 0.180F, 0.450F};
constexpr furuta::SwingSettings kSwingSettingLimits{
    20.00F, 1.000F, 1.000F, 0.300F, 0.450F};
constexpr uint32_t kSwingStartupKickPhaseMs = 180;
// A guarded run may begin away from center. ODrive's filtered position mode
// moves the arm to its saved zero with the hardware-tested gains below, then
// continues holding that exact target until the mechanism is quiet.
constexpr float kSwingPrepositionStartArmAngleRad = 1.75F;
constexpr float kSwingPrepositionStartArmRateRadS = 0.50F;
constexpr float kSwingPrepositionStartDownToleranceRad = 0.80F;
constexpr float kSwingPrepositionStartPendulumRateRadS = 2.0F;
constexpr float kSwingCenterInputFilterBandwidth = 20.0F;
constexpr float kSwingCenterVelocityLimitTurnsS = 0.50F;
constexpr float kSwingCenterPositionGain = 20.0F;
constexpr float kSwingCenterVelocityGain = 0.167F;
constexpr float kSwingCenterVelocityIntegratorGain = 0.333F;
constexpr float kSwingCenterTorqueLimitNm = 0.300F;
constexpr uint32_t kSwingCenterWatchdogFeedPeriodMs = 20;
constexpr float kSwingCenterAngleToleranceRad = 0.020F;
constexpr float kSwingCenterRateToleranceRadS = 0.050F;
constexpr uint32_t kSwingCenterTimeoutMs = 12000;
constexpr float kSwingSettleDownToleranceRad = 0.040F;
constexpr float kSwingSettlePendulumRateRadS = 0.120F;
// The 25 Hz balance-rate estimate intentionally reacts quickly, but a single
// AS5048A count at 200 Hz is already about 0.077 rad/s. A separate slow rate
// estimate prevents stationary count jitter from resetting the 2.5 s settle
// timer while still passing the measured ~1.4 Hz pendulum oscillation.
constexpr float kSwingSettleVelocityFilterHz = 3.0F;
// ODrive's instantaneous velocity estimate showed 0.08 rad/s spikes while
// logged arm travel stayed within 0.00122 rad peak-to-peak. Filter only the
// settle gate; torque control continues using the original ODrive estimate.
constexpr float kSwingSettleArmVelocityFilterHz = 5.0F;
constexpr uint32_t kSwingSettleHoldMs = 2500;
constexpr uint32_t kSwingSettleTimeoutMs = 20000;
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
                  kODriveConfiguredCurrentHardMaxAmp &&
                  kSwingPhaseCurrentLimitAmp <
                      kODriveConfiguredCurrentHardMaxAmp &&
                  kSwingPhaseCurrentLimitAmp <=
                      kODriveConfiguredCurrentSoftMaxAmp,
              "firmware current envelopes must stay inside ODrive limits");
static_assert(kTuningPhaseCurrentLimitAmp <=
                      kCommissioningPhaseCurrentLimitAmp &&
                  kSwingTuningPhaseCurrentLimitAmp <=
                      kSwingPhaseCurrentLimitAmp,
              "current limit tiers must be ordered");
static_assert(kSwingCenterTorqueLimitNm <= kSwingTuningTorqueLimitNm,
              "automatic centering must not exceed swing tuning torque");
static_assert(kSwingSettingLimits.startup_kick_nm <=
                  kSwingTuningTorqueLimitNm &&
                  kSwingSettingLimits.torque_limit_nm <=
                      kSwingTuningTorqueLimitNm,
              "runtime swing settings must remain inside the hard clamp");
static_assert(kSwingPrepositionStartArmAngleRad < kArmAngleLimitRad,
              "swing preparation must begin inside the arm travel limit");
static_assert(kSwingCenterVelocityLimitTurnsS * furuta::kTwoPi <
                  kArmVelocityLimitRadS,
              "automatic centering speed must remain below arm overspeed");
static_assert(kSwingCenterWatchdogFeedPeriodMs < 50U,
              "position hold must feed the 50 ms ODrive watchdog");
static_assert(kODriveHealthRetryMs < kODriveHealthPollMs &&
                  kMaximumConsecutiveHealthQueryErrors > 1U,
              "ODrive health queries must retry quickly before faulting");
static_assert(kSwingCenterInputFilterBandwidth > 0.0F &&
                  kSwingCenterPositionGain > 0.0F &&
                  kSwingCenterVelocityGain > 0.0F &&
                  kSwingCenterVelocityIntegratorGain >= 0.0F,
              "filtered-position gains must be valid");
static_assert(kSwingSettleHoldMs < kSwingSettleTimeoutMs,
              "settling hold must fit inside settling timeout");
static_assert(kSwingSettleVelocityFilterHz > 0.0F &&
                  kSwingSettleVelocityFilterHz < kVelocityFilterHz,
              "settling velocity filter must be slower than balance feedback");
static_assert(kSwingSettleArmVelocityFilterHz > 0.0F &&
                  kSwingSettleArmVelocityFilterHz < kVelocityFilterHz,
              "arm settling filter must be slower than balance feedback");
static_assert(kSwingSettleDownToleranceRad <
                  kSwingPrepositionStartDownToleranceRad &&
                  kSwingSettlePendulumRateRadS <
                      kSwingPrepositionStartPendulumRateRadS,
              "settling gates must be tighter than pre-arm gates");
static_assert(kPendulumDirection == 1.0F || kPendulumDirection == -1.0F,
              "kPendulumDirection must be exactly 1 or -1");
static_assert(kUartResponseTimeoutUs < kControlPeriodUs,
              "UART response timeout must be shorter than the loop period");
static_assert(kPositionFeedbackPeriodMs * 1000U <
                  kPositionFeedbackTimeoutUs &&
                  kPositionFeedbackResponseTimeoutUs <
                      kPositionFeedbackTimeoutUs,
              "position feedback timing must fit its freshness limit");
static_assert(kPositionFeedbackTimeoutUs < 50000U,
              "position feedback must fail before the ODrive watchdog");
static_assert(kUartConfigurationResponseTimeoutUs < 50000U,
              "arming verification timeout must stay below the ODrive watchdog");

}  // namespace config
