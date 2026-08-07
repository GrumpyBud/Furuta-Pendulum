#include <Arduino.h>
#include <SPI.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "as5048a_protocol.hpp"
#include "config.hpp"
#include "control_math.hpp"
#include "uart_protocol.hpp"

namespace {

enum class Mode : uint8_t {
  kDisarmed,
  kTest,
  kSwingCentering,
  kSwingSettling,
  kSwingUp,
  kBalance,
  kTuning,
  kFault,
};

enum class FaultReferencePolicy : uint8_t {
  kPreserve,
  kInvalidate,
};

class ODriveUart {
 public:
  explicit ODriveUart(HardwareSerial& serial) : serial_(serial) {}

  void begin() { serial_.begin(config::kODriveUartBaud); }

  bool send(const char* command) {
    char framed[112]{};
    const int body_length = std::snprintf(framed, sizeof(framed), "%s ", command);
    if (body_length <= 0 || static_cast<size_t>(body_length) >= sizeof(framed)) {
      return false;
    }
    const uint8_t sum =
        odrive_ascii::checksum(framed, static_cast<size_t>(body_length));
    const size_t remaining =
        sizeof(framed) - static_cast<size_t>(body_length);
    const int total_length = std::snprintf(
        framed + body_length, remaining,
        "*%u\n", static_cast<unsigned>(sum));
    if (total_length <= 0 ||
        static_cast<size_t>(total_length) >= remaining) return false;
    return serial_.write(reinterpret_cast<const uint8_t*>(framed),
                         static_cast<size_t>(body_length + total_length)) ==
           static_cast<size_t>(body_length + total_length);
  }

  bool request(const char* command, char* response, const size_t response_size,
               const uint32_t timeout_us = config::kUartResponseTimeoutUs) {
    drainInput();
    if (!send(command)) return false;
    size_t length = 0;
    const uint32_t started_us = micros();
    while (micros() - started_us < timeout_us) {
      while (serial_.available() > 0) {
        const char incoming = static_cast<char>(serial_.read());
        if (incoming == '\r') continue;
        if (incoming == '\n') {
          if (length == 0U) continue;
          response[length] = '\0';
          return odrive_ascii::validateAndStripChecksum(response);
        }
        if (length + 1U >= response_size) {
          drainInput();
          return false;
        }
        response[length++] = incoming;
      }
    }
    return false;
  }

  bool feedback(float& position_turns, float& velocity_turns_s) {
    char command[12]{};
    const int written =
        std::snprintf(command, sizeof(command), "f %u", config::kODriveAxis);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    char response[64]{};
    return request(command, response, sizeof(response)) &&
           odrive_ascii::parseFeedback(response, position_turns,
                                       velocity_turns_s);
  }

  bool readUnsigned(const char* property, uint32_t& value,
                    const uint32_t timeout_us =
                        config::kUartResponseTimeoutUs) {
    char command[80]{};
    const int written =
        std::snprintf(command, sizeof(command), "r %s", property);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    char response[48]{};
    if (!request(command, response, sizeof(response), timeout_us)) return false;
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(response, &end, 0);
    while (end != nullptr && *end == ' ') ++end;
    if (end == response || end == nullptr || *end != '\0') return false;
    value = static_cast<uint32_t>(parsed);
    return true;
  }

  bool readFloat(const char* property, float& value,
                 const uint32_t timeout_us =
                     config::kUartResponseTimeoutUs) {
    char command[80]{};
    const int written = std::snprintf(command, sizeof(command), "r %s",
                                      property);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    char response[48]{};
    if (!request(command, response, sizeof(response), timeout_us)) return false;
    char* end = nullptr;
    value = std::strtof(response, &end);
    while (end != nullptr && *end == ' ') ++end;
    return end != response && end != nullptr && *end == '\0' &&
           std::isfinite(value);
  }

  bool setTorque(const float torque_nm) {
    char command[40]{};
    const int written =
        std::snprintf(command, sizeof(command), "c %u %.6f",
                      config::kODriveAxis, static_cast<double>(torque_nm));
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    return send(command);
  }

  bool setPosition(const float position_turns,
                   const float velocity_limit_turns_s,
                   const float torque_limit_nm) {
    char command[64]{};
    const int written = std::snprintf(
        command, sizeof(command), "q %u %.7f %.6f %.6f",
        config::kODriveAxis, static_cast<double>(position_turns),
        static_cast<double>(velocity_limit_turns_s),
        static_cast<double>(torque_limit_nm));
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    return send(command);
  }

  bool writeProperty(const char* property, const char* value) {
    char command[112]{};
    const int written = std::snprintf(command, sizeof(command), "w %s %s",
                                      property, value);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    return send(command);
  }

  bool writeFloatProperty(const char* property, const float value) {
    char text[24]{};
    const int written = std::snprintf(text, sizeof(text), "%.7g",
                                      static_cast<double>(value));
    return written > 0 && static_cast<size_t>(written) < sizeof(text) &&
           writeProperty(property, text);
  }

  bool clearErrors() { return send("sc"); }

  void drainInput() {
    while (serial_.available() > 0) static_cast<void>(serial_.read());
  }

 private:
  HardwareSerial& serial_;
};

class PendulumEncoder {
 public:
  void begin() {
    pinMode(config::kPendulumChipSelectPin, OUTPUT);
    digitalWrite(config::kPendulumChipSelectPin, HIGH);
    SPI.begin();
    delay(20);
    clearError();
  }

  bool readAngle(uint16_t& count) {
    return readRegister(as5048a::kRegisterAngle, count);
  }

  bool readDiagnostics(as5048a::Diagnostics& diagnostics) {
    uint16_t raw = 0;
    if (!readRegister(as5048a::kRegisterDiagnostics, raw)) return false;
    diagnostics = as5048a::decodeDiagnostics(raw);
    return true;
  }

  uint32_t parityErrors() const { return parity_errors_; }
  uint32_t protocolErrors() const { return protocol_errors_; }

 private:
  uint16_t transfer(const uint16_t value) {
    const SPISettings settings(config::kEncoderSpiHz, MSBFIRST, SPI_MODE1);
    SPI.beginTransaction(settings);
    digitalWrite(config::kPendulumChipSelectPin, LOW);
    delayNanoseconds(400);
    const uint16_t response = SPI.transfer16(value);
    delayNanoseconds(100);
    digitalWrite(config::kPendulumChipSelectPin, HIGH);
    SPI.endTransaction();
    delayNanoseconds(400);
    return response;
  }

  void clearError() {
    transfer(as5048a::makeReadCommand(as5048a::kRegisterClearError));
    transfer(as5048a::kNopCommand);
  }

  bool readRegister(const uint16_t address, uint16_t& value) {
    transfer(as5048a::makeReadCommand(address));
    const as5048a::Response response =
        as5048a::parseResponse(transfer(as5048a::kNopCommand));
    if (!response.parity_ok) {
      ++parity_errors_;
      return false;
    }
    if (response.error_flag) {
      ++protocol_errors_;
      clearError();
      return false;
    }
    value = response.data;
    return true;
  }

  uint32_t parity_errors_ = 0;
  uint32_t protocol_errors_ = 0;
};

ODriveUart odrive(Serial1);
PendulumEncoder encoder;
Mode mode = Mode::kDisarmed;
furuta::State state{};
furuta::Gains gains = config::kDefaultBalanceGains;
furuta::SwingSettings swing_settings = config::kDefaultSwingSettings;
as5048a::Diagnostics encoder_diagnostics{};
uint16_t pendulum_count = 0;
uint16_t pendulum_down_count = 0;
float arm_position_turns = 0.0F;
float arm_velocity_turns_s = 0.0F;
float arm_zero_turns = 0.0F;
float last_pendulum_angle = -furuta::kPi;
float pendulum_velocity = 0.0F;
float commanded_torque_nm = 0.0F;
uint32_t last_feedback_us = 0;
uint32_t next_control_us = 0;
uint32_t last_inactive_feedback_ms = 0;
uint32_t last_health_poll_ms = 0;
uint32_t last_diagnostics_ms = 0;
uint32_t last_telemetry_ms = 0;
uint32_t tuning_started_ms = 0;
uint32_t run_started_ms = 0;
uint32_t swing_phase_started_ms = 0;
uint32_t swing_settle_ready_ms = 0;
uint32_t last_position_command_ms = 0;
uint32_t browser_deadman_ms = 0;
uint32_t last_state_sample_us = 0;
uint32_t consecutive_encoder_errors = 0;
uint32_t consecutive_health_query_errors = 0;
uint32_t consecutive_deadline_misses = 0;
uint32_t last_tick_duration_us = 0;
uint32_t maximum_tick_duration_us = 0;
uint32_t odrive_active_errors = 0;
bool zero_valid = false;
bool odrive_online = false;
bool encoder_healthy = false;
bool browser_deadman_held = false;
bool guarded_swing_trial = false;
char fault_reason[112]{};
char odrive_arm_failure[112]{};
char command_buffer[128]{};
size_t command_length = 0;

bool commandODriveCenter(uint32_t now_ms);
bool switchODriveToTorqueControl();

bool isActive() {
  return mode == Mode::kSwingCentering || mode == Mode::kSwingSettling ||
         mode == Mode::kSwingUp || mode == Mode::kBalance ||
         mode == Mode::kTuning;
}

bool browserDeadmanFresh(const uint32_t now_ms) {
  return furuta::keepaliveFresh(browser_deadman_held, now_ms,
                                browser_deadman_ms,
                                config::kBrowserDeadmanTimeoutMs);
}

const char* modeName() {
  switch (mode) {
    case Mode::kDisarmed: return "DISARMED";
    case Mode::kTest: return "TEST";
    case Mode::kSwingCentering: return "CENTERING";
    case Mode::kSwingSettling: return "SETTLING";
    case Mode::kSwingUp: return "SWING_UP";
    case Mode::kBalance: return "BALANCE";
    case Mode::kTuning: return "TUNING";
    case Mode::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

void event(const char* level, const char* code, const char* message) {
  Serial.print(F("@E,"));
  Serial.print(level);
  Serial.print(',');
  Serial.print(code);
  Serial.print(',');
  Serial.println(message);
}

void requestIdle() {
  static_cast<void>(odrive.setTorque(0.0F));
  static_cast<void>(
      odrive.writeProperty("axis0.requested_state", "1"));  // IDLE
  // If UART is healthy, do not leave a previously armed watchdog running while
  // deliberately idle. If UART is broken these writes cannot arrive, so the
  // already-running ODrive watchdog still provides the intended fallback.
  static_cast<void>(
      odrive.writeProperty("axis0.config.enable_watchdog", "0"));
  commanded_torque_nm = 0.0F;
}

void enterFault(
    const char* reason,
    const FaultReferencePolicy reference_policy =
        FaultReferencePolicy::kPreserve) {
  if (mode == Mode::kFault) return;
  requestIdle();
  mode = Mode::kFault;
  guarded_swing_trial = false;
  const bool invalidated_zero =
      zero_valid &&
      reference_policy == FaultReferencePolicy::kInvalidate;
  if (invalidated_zero) zero_valid = false;
  std::snprintf(fault_reason, sizeof(fault_reason), "%s", reason);
  event("ERROR", "FAULT", fault_reason);
  if (invalidated_zero) {
    event("WARN", "ZERO_INVALIDATED",
          "sensor or drive-reference integrity was lost; save zero again");
  }
}

bool readODriveFeedback() {
  if (!odrive.feedback(arm_position_turns, arm_velocity_turns_s)) {
    odrive_online = false;
    return false;
  }
  odrive_online = true;
  last_feedback_us = micros();
  return true;
}

bool sampleEncoder(const float dt_s) {
  uint16_t new_count = 0;
  if (!encoder.readAngle(new_count)) {
    ++consecutive_encoder_errors;
    return false;
  }
  consecutive_encoder_errors = 0;
  pendulum_count = new_count;
  const int32_t delta_count = static_cast<int32_t>(pendulum_count) -
                              static_cast<int32_t>(pendulum_down_count);
  const float count_to_rad = config::kPendulumDirection * furuta::kTwoPi /
                             as5048a::kCountsPerRevolution;
  const float pendulum_angle =
      furuta::wrapAngle(delta_count * count_to_rad - furuta::kPi);
  const float raw_velocity =
      furuta::wrapAngle(pendulum_angle - last_pendulum_angle) / dt_s;
  pendulum_velocity = furuta::lowPass(
      pendulum_velocity, raw_velocity, config::kVelocityFilterHz, dt_s);
  last_pendulum_angle = pendulum_angle;
  const float arm_scale =
      config::kMotorTurnsToArmRadians * config::kMotorDirection;
  state = {(arm_position_turns - arm_zero_turns) * arm_scale, pendulum_angle,
           arm_velocity_turns_s * arm_scale, pendulum_velocity};
  return true;
}

const char* encoderStatus() {
  if (!encoder_diagnostics.offset_compensation_finished) return "OCF_NOT_READY";
  if (encoder_diagnostics.cordic_overflow) return "CORDIC_OVERFLOW";
  if (encoder_diagnostics.magnetic_field_too_weak) return "MAGNET_WEAK";
  if (encoder_diagnostics.magnetic_field_too_strong) return "MAGNET_STRONG";
  return encoder_healthy ? "OK" : "READ_ERROR";
}

bool stateIsHealthy(const uint32_t now_us) {
  if (!browserDeadmanFresh(millis())) {
    enterFault("browser Space dead-man released or page lost focus");
  } else if (!encoder_healthy ||
             consecutive_encoder_errors >=
                 config::kMaximumConsecutiveEncoderErrors) {
    enterFault("AS5048A encoder unhealthy",
               FaultReferencePolicy::kInvalidate);
  } else if (!odrive_online ||
             now_us - last_feedback_us > config::kFeedbackTimeoutUs) {
    enterFault("ODrive UART feedback timeout",
               FaultReferencePolicy::kInvalidate);
  } else if (odrive_active_errors != 0U) {
    enterFault("ODrive reports an active error",
               FaultReferencePolicy::kInvalidate);
  } else if (!std::isfinite(state.arm_angle_rad) ||
             !std::isfinite(state.pendulum_angle_rad) ||
             !std::isfinite(state.arm_velocity_rad_s) ||
             !std::isfinite(state.pendulum_velocity_rad_s)) {
    enterFault("non-finite sensor value",
               FaultReferencePolicy::kInvalidate);
  } else if (furuta::projectedAbsoluteTravel(
                 state.arm_angle_rad, state.arm_velocity_rad_s,
                 config::kArmTravelPredictionSeconds) >
             config::kArmAngleLimitRad) {
    enterFault("arm travel stopping margin");
  } else if (std::fabs(state.arm_velocity_rad_s) >
             config::kArmVelocityLimitRadS) {
    enterFault("arm overspeed");
  } else if (std::fabs(state.pendulum_velocity_rad_s) >
             config::kPendulumVelocityLimitRadS) {
    enterFault("pendulum overspeed");
  }
  return mode != Mode::kFault;
}

void refreshDiagnostics(const uint32_t now_ms) {
  if (now_ms - last_diagnostics_ms < config::kEncoderDiagnosticsPeriodMs) {
    return;
  }
  last_diagnostics_ms = now_ms;
  if (encoder.readDiagnostics(encoder_diagnostics)) {
    encoder_healthy = encoder_diagnostics.healthy();
  } else {
    encoder_healthy = false;
    ++consecutive_encoder_errors;
  }
}

void pollODriveHealth(const uint32_t now_ms) {
  last_health_poll_ms = now_ms;
  uint32_t errors = 0;
  if (!odrive.readUnsigned("axis0.active_errors", errors)) {
    if (isActive()) {
      ++consecutive_health_query_errors;
      if (consecutive_health_query_errors >=
          config::kMaximumConsecutiveHealthQueryErrors) {
        // Successful cyclic feedback between these property-query attempts
        // proves that the position reference still exists. Stop motion, but
        // preserve zero; a real feedback loss has its own immediate fault.
        enterFault("ODrive health query failed after rapid retries");
      } else {
        // Pull the next health poll forward without blocking normal feedback
        // on the intervening 5 ms control tick.
        last_health_poll_ms =
            now_ms - config::kODriveHealthPollMs +
            config::kODriveHealthRetryMs;
      }
    } else {
      odrive_online = false;
      consecutive_health_query_errors = 0U;
    }
    return;
  }
  consecutive_health_query_errors = 0U;
  odrive_active_errors = errors;
  odrive_online = true;
}

void runControlTick() {
  const uint32_t now_ms = millis();
  refreshDiagnostics(now_ms);

  // A health-property request replaces one feedback request periodically. The
  // previous arm sample remains safely inside the feedback age limit.
  const uint32_t health_interval_ms =
      isActive() ? config::kODriveHealthPollMs
                 : config::kODriveInactiveHealthPollMs;
  const bool health_poll_due =
      now_ms - last_health_poll_ms >= health_interval_ms;
  if (health_poll_due) {
    pollODriveHealth(now_ms);
  } else if (isActive()) {
    if (!readODriveFeedback()) {
      enterFault("ODrive UART feedback failed",
                 FaultReferencePolicy::kInvalidate);
    }
  } else if (now_ms - last_inactive_feedback_ms >= 20U) {
    last_inactive_feedback_ms = now_ms;
    static_cast<void>(readODriveFeedback());
  }

  const uint32_t sample_us = micros();
  float dt_s = config::kControlPeriodUs * 1.0e-6F;
  if (last_state_sample_us != 0U) {
    dt_s = furuta::clamp((sample_us - last_state_sample_us) * 1.0e-6F,
                         0.001F, 0.025F);
  }
  last_state_sample_us = sample_us;
  if (!sampleEncoder(dt_s)) {
    if (isActive() && consecutive_encoder_errors >=
                          config::kMaximumConsecutiveEncoderErrors) {
      enterFault("AS5048A repeated read errors",
                 FaultReferencePolicy::kInvalidate);
    }
    return;
  }

  if (!isActive() || !stateIsHealthy(micros())) return;

  float requested_torque = 0.0F;
  if (mode == Mode::kTuning) {
    if (now_ms - tuning_started_ms > config::kTuningMaximumRunMs) {
      enterFault("tuning time limit reached");
      return;
    }
    if (std::fabs(state.pendulum_angle_rad) > config::kTuningAbortAngleRad) {
      enterFault("pendulum left tuning catch region");
      return;
    }
    requested_torque = furuta::clamp(
        furuta::balanceTorque(state, gains), -config::kTuningTorqueLimitNm,
        config::kTuningTorqueLimitNm);
  } else if (mode == Mode::kSwingCentering) {
    if (now_ms - swing_phase_started_ms > config::kSwingCenterTimeoutMs) {
      enterFault("automatic arm centering time limit reached");
      return;
    }
    if (!health_poll_due && !commandODriveCenter(now_ms)) {
      enterFault("ODrive filtered-position command failed",
                 FaultReferencePolicy::kInvalidate);
      return;
    }
    commanded_torque_nm = 0.0F;
    if (std::fabs(state.arm_angle_rad) <
            config::kSwingCenterAngleToleranceRad &&
        std::fabs(state.arm_velocity_rad_s) <
            config::kSwingCenterRateToleranceRadS) {
      mode = Mode::kSwingSettling;
      swing_phase_started_ms = now_ms;
      swing_settle_ready_ms = 0U;
      event("INFO", "SWING_PHASE", "arm centered; waiting for pendulum to settle");
    }
    return;
  } else if (mode == Mode::kSwingSettling) {
    if (now_ms - swing_phase_started_ms > config::kSwingSettleTimeoutMs) {
      enterFault("pendulum settling time limit reached");
      return;
    }
    if (!health_poll_due && !commandODriveCenter(now_ms)) {
      enterFault("ODrive position-hold command failed",
                 FaultReferencePolicy::kInvalidate);
      return;
    }
    commanded_torque_nm = 0.0F;
    const bool settled = furuta::swingPreparationSettled(
        state, config::kSwingCenterAngleToleranceRad,
        config::kSwingCenterRateToleranceRadS,
        config::kSwingSettleDownToleranceRad,
        config::kSwingSettlePendulumRateRadS);
    if (!settled) {
      swing_settle_ready_ms = 0U;
    } else if (swing_settle_ready_ms == 0U) {
      swing_settle_ready_ms = now_ms;
    } else if (now_ms - swing_settle_ready_ms >=
               config::kSwingSettleHoldMs) {
      if (!switchODriveToTorqueControl()) {
        enterFault(odrive_arm_failure[0] == '\0'
                       ? "ODrive refused torque-mode handoff"
                       : odrive_arm_failure,
                   FaultReferencePolicy::kInvalidate);
        return;
      }
      mode = Mode::kSwingUp;
      run_started_ms = now_ms;
      event("WARN", "SWING_PHASE", "pendulum settled; energy swing started");
    }
    return;
  } else {
    if (guarded_swing_trial &&
        now_ms - run_started_ms > config::kSwingTuningMaximumRunMs) {
      enterFault("swing-up tuning time limit reached");
      return;
    }
    if (mode == Mode::kSwingUp &&
        std::fabs(state.pendulum_angle_rad) < config::kCatchAngleRad &&
        std::fabs(state.pendulum_velocity_rad_s) <
            config::kCatchPendulumVelocityRadS) {
      mode = Mode::kBalance;
      event("INFO", "MODE", "balance catch");
    } else if (mode == Mode::kBalance &&
               std::fabs(state.pendulum_angle_rad) > config::kDropAngleRad) {
      mode = Mode::kSwingUp;
      event("WARN", "MODE", "returned to swing-up");
    }

    if (mode == Mode::kBalance) {
      requested_torque = furuta::balanceTorque(state, gains);
    } else {
      const uint32_t startup_elapsed_ms = now_ms - run_started_ms;
      if (startup_elapsed_ms < config::kSwingStartupKickPhaseMs) {
        requested_torque = swing_settings.startup_kick_nm;
      } else {
        requested_torque = furuta::swingUpTorque(
            state, config::kPendulumMassKg, config::kPendulumComLengthM,
            config::kPendulumInertiaKgM2, swing_settings.energy_gain,
            swing_settings.arm_damping);
      }
      requested_torque -=
          swing_settings.arm_centering * state.arm_angle_rad;
      const float active_swing_limit_nm = std::fmin(
          swing_settings.torque_limit_nm, config::kSwingTorqueLimitNm);
      requested_torque = furuta::clamp(
          requested_torque, -active_swing_limit_nm,
          active_swing_limit_nm);
    }
    if (guarded_swing_trial) {
      requested_torque = furuta::clamp(
          requested_torque, -config::kSwingTuningTorqueLimitNm,
          config::kSwingTuningTorqueLimitNm);
    }
    requested_torque = furuta::clamp(
        requested_torque, -config::kTorqueLimitNm, config::kTorqueLimitNm);
  }

  commanded_torque_nm = furuta::slewLimit(
      commanded_torque_nm, requested_torque,
      config::kTorqueSlewNmPerSecond * dt_s);
  // State and gains use the configured positive arm coordinate. Convert the
  // logical torque back into the ODrive encoder/motor coordinate here.
  if (!odrive.setTorque(commanded_torque_nm * config::kMotorDirection)) {
    enterFault("ODrive UART torque write failed",
               FaultReferencePolicy::kInvalidate);
  }
}

bool gainsAreAllowed(const furuta::Gains& candidate) {
  return furuta::gainsWithinAbsoluteLimits(candidate,
                                           config::kGainAbsoluteLimits) &&
         furuta::gainsWithinScaledProfile(
             candidate, config::kModelBalanceGains,
             config::kMinimumRuntimeGainScale,
             config::kMaximumRuntimeGainScale);
}

bool parseStrictFloat(const char* text, float& value) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0' &&
         std::isfinite(value);
}

bool nearlyEqual(const float actual, const float expected,
                 const float tolerance = 0.001F) {
  return std::isfinite(actual) && std::fabs(actual - expected) <= tolerance;
}

bool commandODriveCenter(const uint32_t now_ms) {
  if (now_ms - last_position_command_ms <
      config::kSwingCenterCommandPeriodMs) {
    return true;
  }
  if (!odrive.setPosition(arm_zero_turns,
                          config::kSwingCenterVelocityLimitTurnsS,
                          config::kSwingCenterTorqueLimitNm)) {
    return false;
  }
  last_position_command_ms = now_ms;
  return true;
}

bool switchODriveToTorqueControl() {
  odrive_arm_failure[0] = '\0';
  const auto fail = [](const char* reason) {
    std::snprintf(odrive_arm_failure, sizeof(odrive_arm_failure), "%s",
                  reason);
    requestIdle();
    return false;
  };

  // PASSTHROUGH is valid for position and torque modes, so change it first.
  // The official ASCII `c` command then changes control mode to torque and
  // feeds the already-running ODrive watchdog with an explicit zero request.
  if (!odrive.writeProperty("axis0.controller.config.input_mode", "1")) {
    return fail("ODrive rejected torque passthrough during swing handoff");
  }
  if (!odrive.setTorque(0.0F)) {
    return fail("ODrive rejected zero torque during swing handoff");
  }

  uint32_t control_mode = 0;
  uint32_t input_mode = 0;
  if (!odrive.readUnsigned("axis0.controller.config.control_mode",
                           control_mode,
                           config::kUartConfigurationResponseTimeoutUs) ||
      !odrive.setTorque(0.0F)) {
    return fail("ODrive did not verify torque mode during swing handoff");
  }
  if (!odrive.readUnsigned("axis0.controller.config.input_mode", input_mode,
                           config::kUartConfigurationResponseTimeoutUs) ||
      !odrive.setTorque(0.0F)) {
    return fail("ODrive did not verify passthrough during swing handoff");
  }
  if (control_mode != 1U || input_mode != 1U) {
    return fail("ODrive mode mismatch during swing handoff");
  }
  commanded_torque_nm = 0.0F;
  return true;
}

bool armODriveTorque() {
  odrive_arm_failure[0] = '\0';
  const auto fail = [](const char* reason) {
    std::snprintf(odrive_arm_failure, sizeof(odrive_arm_failure), "%s",
                  reason);
    requestIdle();
    return false;
  };

  odrive.drainInput();
  // Disable a watchdog left over from an earlier interrupted trial before
  // clearing errors. Enabling it before these UART verification transactions
  // can make the 50 ms timer expire during the arming procedure itself.
  if (!odrive.writeProperty("axis0.config.enable_watchdog", "0")) {
    return fail("ODrive UART failed while disabling the stale watchdog");
  }
  if (!odrive.clearErrors()) {
    return fail("ODrive UART failed while clearing errors before arming");
  }
  if (!odrive.writeProperty("axis0.config.watchdog_timeout",
                            config::kODriveWatchdogSeconds)) {
    return fail("ODrive rejected the watchdog timeout");
  }
  if (!odrive.writeProperty("axis0.controller.config.input_mode", "1")) {
    return fail("ODrive rejected torque passthrough input mode");
  }
  if (!odrive.writeProperty("axis0.controller.config.control_mode", "1")) {
    return fail("ODrive rejected torque control mode");
  }
  if (!odrive.setTorque(0.0F)) {
    return fail("ODrive UART failed while writing zero torque");
  }
  if (!odrive.writeProperty("axis0.requested_state", "8")) {
    return fail("ODrive UART failed while requesting closed-loop state");
  }

  delay(30);
  uint32_t watchdog_enabled = 0;
  uint32_t control_mode = 0;
  uint32_t input_mode = 0;
  uint32_t current_state = 0;
  uint32_t active_errors = 0;
  if (!odrive.readUnsigned("axis0.controller.config.control_mode",
                           control_mode,
                           config::kUartConfigurationResponseTimeoutUs)) {
    return fail("ODrive did not answer the control-mode verification query");
  }
  if (!odrive.readUnsigned("axis0.controller.config.input_mode", input_mode,
                           config::kUartConfigurationResponseTimeoutUs)) {
    return fail("ODrive did not answer the input-mode verification query");
  }
  if (!odrive.readUnsigned("axis0.current_state", current_state,
                           config::kUartConfigurationResponseTimeoutUs)) {
    return fail("ODrive did not answer the axis-state verification query");
  }
  if (!odrive.readUnsigned("axis0.active_errors", active_errors,
                           config::kUartConfigurationResponseTimeoutUs)) {
    return fail("ODrive did not answer the active-error verification query");
  }

  odrive_active_errors = active_errors;
  commanded_torque_nm = 0.0F;
  if (active_errors != 0U || current_state != 8U) {
    std::snprintf(odrive_arm_failure, sizeof(odrive_arm_failure),
                  "ODrive closed-loop refusal: state %lu, errors 0x%08lX",
                  static_cast<unsigned long>(current_state),
                  static_cast<unsigned long>(active_errors));
    requestIdle();
    return false;
  }
  if (control_mode != 1U || input_mode != 1U) {
    std::snprintf(odrive_arm_failure, sizeof(odrive_arm_failure),
                  "ODrive mode mismatch: control %lu, input %lu",
                  static_cast<unsigned long>(control_mode),
                  static_cast<unsigned long>(input_mode));
    requestIdle();
    return false;
  }

  // Start the short hardware watchdog only after closed-loop state and the
  // zero-torque modes are proven. Feed it immediately before further reads.
  if (!odrive.writeProperty("axis0.config.enable_watchdog", "1") ||
      !odrive.setTorque(0.0F)) {
    return fail("ODrive UART failed while starting the verified watchdog");
  }
  if (!odrive.readUnsigned("axis0.config.enable_watchdog",
                           watchdog_enabled,
                           config::kUartConfigurationResponseTimeoutUs)) {
    return fail("ODrive did not answer the watchdog verification query");
  }
  if (watchdog_enabled != 1U) {
    return fail("ODrive watchdog did not enable after closed-loop entry");
  }
  if (!readODriveFeedback()) {
    return fail("ODrive feedback failed immediately after closed-loop entry");
  }
  if (!odrive.setTorque(0.0F)) {
    return fail("ODrive zero-torque watchdog feed failed after arming");
  }
  return true;
}

bool armODrivePosition() {
  odrive_arm_failure[0] = '\0';
  const auto fail = [](const char* reason) {
    std::snprintf(odrive_arm_failure, sizeof(odrive_arm_failure), "%s",
                  reason);
    requestIdle();
    return false;
  };

  odrive.drainInput();
  if (!odrive.writeProperty("axis0.config.enable_watchdog", "0")) {
    return fail("ODrive UART failed while disabling the stale watchdog");
  }
  if (!odrive.clearErrors()) {
    return fail("ODrive UART failed while clearing errors before centering");
  }
  if (!odrive.writeProperty("axis0.config.watchdog_timeout",
                            config::kODriveWatchdogSeconds)) {
    return fail("ODrive rejected the watchdog timeout");
  }
  if (!odrive.writeFloatProperty(
          "axis0.controller.config.input_filter_bandwidth",
          config::kSwingCenterInputFilterBandwidth) ||
      !odrive.writeFloatProperty("axis0.controller.config.vel_limit",
                                 config::kSwingCenterVelocityLimitTurnsS) ||
      !odrive.writeProperty("axis0.controller.config.enable_vel_limit", "1") ||
      !odrive.writeFloatProperty("axis0.controller.config.pos_gain",
                                 config::kSwingCenterPositionGain) ||
      !odrive.writeFloatProperty("axis0.controller.config.vel_gain",
                                 config::kSwingCenterVelocityGain) ||
      !odrive.writeFloatProperty(
          "axis0.controller.config.vel_integrator_gain",
          config::kSwingCenterVelocityIntegratorGain)) {
    return fail("ODrive rejected filtered-position gains or limits");
  }
  if (!odrive.writeProperty("axis0.controller.config.input_mode", "3") ||
      !odrive.writeProperty("axis0.controller.config.control_mode", "3")) {
    return fail("ODrive rejected filtered position control mode");
  }

  // Seed the filtered setpoint at the measured position before closed-loop
  // entry. Only after the mode and gains are verified do we command saved zero.
  if (!odrive.setPosition(arm_position_turns,
                          config::kSwingCenterVelocityLimitTurnsS,
                          config::kSwingCenterTorqueLimitNm) ||
      !odrive.writeProperty("axis0.requested_state", "8")) {
    return fail("ODrive refused filtered-position closed-loop entry");
  }
  delay(30);

  uint32_t control_mode = 0;
  uint32_t input_mode = 0;
  uint32_t velocity_limit_enabled = 0;
  uint32_t current_state = 0;
  uint32_t active_errors = 0;
  float filter_bandwidth = 0.0F;
  float velocity_limit = 0.0F;
  float position_gain = 0.0F;
  float velocity_gain = 0.0F;
  float velocity_integrator_gain = 0.0F;
  const uint32_t timeout = config::kUartConfigurationResponseTimeoutUs;
  if (!odrive.readUnsigned("axis0.controller.config.control_mode",
                           control_mode, timeout) ||
      !odrive.readUnsigned("axis0.controller.config.input_mode", input_mode,
                           timeout) ||
      !odrive.readUnsigned("axis0.controller.config.enable_vel_limit",
                           velocity_limit_enabled, timeout) ||
      !odrive.readUnsigned("axis0.current_state", current_state, timeout) ||
      !odrive.readUnsigned("axis0.active_errors", active_errors, timeout) ||
      !odrive.readFloat("axis0.controller.config.input_filter_bandwidth",
                        filter_bandwidth, timeout) ||
      !odrive.readFloat("axis0.controller.config.vel_limit", velocity_limit,
                        timeout) ||
      !odrive.readFloat("axis0.controller.config.pos_gain", position_gain,
                        timeout) ||
      !odrive.readFloat("axis0.controller.config.vel_gain", velocity_gain,
                        timeout) ||
      !odrive.readFloat("axis0.controller.config.vel_integrator_gain",
                        velocity_integrator_gain, timeout)) {
    return fail("ODrive did not verify filtered-position configuration");
  }
  odrive_active_errors = active_errors;
  if (active_errors != 0U || current_state != 8U || control_mode != 3U ||
      input_mode != 3U || velocity_limit_enabled != 1U ||
      !nearlyEqual(filter_bandwidth,
                   config::kSwingCenterInputFilterBandwidth) ||
      !nearlyEqual(velocity_limit,
                   config::kSwingCenterVelocityLimitTurnsS) ||
      !nearlyEqual(position_gain, config::kSwingCenterPositionGain) ||
      !nearlyEqual(velocity_gain, config::kSwingCenterVelocityGain) ||
      !nearlyEqual(velocity_integrator_gain,
                   config::kSwingCenterVelocityIntegratorGain)) {
    return fail("ODrive filtered-position readback mismatch");
  }

  if (!odrive.writeProperty("axis0.config.enable_watchdog", "1") ||
      !odrive.setPosition(arm_zero_turns,
                          config::kSwingCenterVelocityLimitTurnsS,
                          config::kSwingCenterTorqueLimitNm)) {
    return fail("ODrive failed to start the centered-position watchdog");
  }
  uint32_t watchdog_enabled = 0;
  float commanded_position = 0.0F;
  if (!odrive.readUnsigned("axis0.config.enable_watchdog", watchdog_enabled,
                           timeout) ||
      !odrive.setPosition(arm_zero_turns,
                          config::kSwingCenterVelocityLimitTurnsS,
                          config::kSwingCenterTorqueLimitNm) ||
      !odrive.readFloat("axis0.controller.input_pos", commanded_position,
                        timeout)) {
    return fail("ODrive did not verify the centered-position target");
  }
  if (watchdog_enabled != 1U ||
      !nearlyEqual(commanded_position, arm_zero_turns, 0.0001F)) {
    return fail("ODrive centered-position target readback mismatch");
  }
  if (!readODriveFeedback() ||
      !odrive.setPosition(arm_zero_turns,
                          config::kSwingCenterVelocityLimitTurnsS,
                          config::kSwingCenterTorqueLimitNm)) {
    return fail("ODrive position feedback failed immediately after arming");
  }
  commanded_torque_nm = 0.0F;
  last_position_command_ms = millis();
  return true;
}

void disarm(const bool acknowledge_fault) {
  requestIdle();
  delay(10);
  uint32_t current_state = 0;
  if (odrive.readUnsigned("axis0.current_state", current_state) &&
      current_state == 1U) {
    static_cast<void>(
        odrive.writeProperty("axis0.config.enable_watchdog", "0"));
  }
  mode = Mode::kDisarmed;
  guarded_swing_trial = false;
  swing_phase_started_ms = 0U;
  swing_settle_ready_ms = 0U;
  last_position_command_ms = 0U;
  consecutive_health_query_errors = 0U;
  commanded_torque_nm = 0.0F;
  if (acknowledge_fault) fault_reason[0] = '\0';
  event("INFO", "DISARMED", "motor requested idle");
}

void zeroSystem() {
  if (mode != Mode::kDisarmed && mode != Mode::kTest) {
    event("WARN", "REFUSED", "zero is allowed only while motor is disabled");
    return;
  }
  if (!encoder_healthy || !odrive_online) {
    event("WARN", "REFUSED", "encoder and ODrive must both be online");
    return;
  }
  if (std::fabs(state.arm_velocity_rad_s) >
          config::kZeroMaximumArmRateRadS ||
      std::fabs(state.pendulum_velocity_rad_s) >
          config::kZeroMaximumPendulumRateRadS) {
    event("WARN", "REFUSED", "hold both arm and pendulum motionless before zero");
    return;
  }
  pendulum_down_count = pendulum_count;
  arm_zero_turns = arm_position_turns;
  last_pendulum_angle = -furuta::kPi;
  pendulum_velocity = 0.0F;
  zero_valid = true;
  event("INFO", "ZEROED", "reference saved; pendulum must have been hanging down");
}

void clearODriveErrors() {
  if (isActive()) {
    event("WARN", "REFUSED", "disarm motor control before clearing ODrive errors");
    return;
  }

  // Keep this action fail-safe: first request zero torque and IDLE, then clear
  // the drive's latched errors, and finally read back the live error mask. A
  // successful UART write alone is not proof that the underlying fault cleared.
  requestIdle();
  delay(10);
  if (!odrive.clearErrors()) {
    odrive_online = false;
    event("ERROR", "ODRIVE_CLEAR_FAILED", "UART write failed while clearing ODrive errors");
    return;
  }

  delay(20);
  uint32_t errors = 0;
  if (!odrive.readUnsigned("axis0.active_errors", errors)) {
    odrive_online = false;
    event("ERROR", "ODRIVE_CLEAR_FAILED", "ODrive did not answer the verification query");
    return;
  }

  odrive_online = true;
  odrive_active_errors = errors;
  if (errors == 0U) {
    event("INFO", "ODRIVE_CLEARED", "ODrive active errors are now clear");
    return;
  }

  char message[80]{};
  std::snprintf(message, sizeof(message),
                "underlying condition remains; active errors 0x%08lX",
                static_cast<unsigned long>(errors));
  event("WARN", "ODRIVE_ERRORS_REMAIN", message);
}

void executeCommand() {
  command_buffer[command_length] = '\0';
  char original[sizeof(command_buffer)]{};
  std::snprintf(original, sizeof(original), "%s", command_buffer);
  char* save = nullptr;
  char* verb = strtok_r(command_buffer, " ", &save);
  if (verb == nullptr) return;

  if (std::strcmp(verb, "disarm") == 0) {
    disarm(true);
  } else if (std::strcmp(verb, "odrive_clear") == 0) {
    clearODriveErrors();
  } else if (std::strcmp(verb, "deadman_hold") == 0) {
    browser_deadman_held = true;
    browser_deadman_ms = millis();
  } else if (std::strcmp(verb, "deadman_release") == 0) {
    browser_deadman_held = false;
    if (isActive()) disarm(false);
  } else if (std::strcmp(verb, "test_start") == 0) {
    if (mode == Mode::kFault) {
      event("WARN", "REFUSED", "acknowledge the fault with disarm first");
    } else if (isActive()) {
      event("WARN", "REFUSED", "disarm before entering sensor test mode");
    } else {
      requestIdle();
      mode = Mode::kTest;
      event("INFO", "TEST", "sensor test mode; motor output is disabled");
    }
  } else if (std::strcmp(verb, "test_stop") == 0) {
    if (mode == Mode::kTest) disarm(false);
  } else if (std::strcmp(verb, "zero") == 0) {
    zeroSystem();
  } else if (std::strcmp(verb, "gains") == 0) {
    if (mode != Mode::kDisarmed && mode != Mode::kTest) {
      event("WARN", "REFUSED", "change gains only while motor is disabled");
      return;
    }
    char* first = strtok_r(nullptr, " ", &save);
    if (first != nullptr && std::strcmp(first, "default") == 0) {
      gains = config::kDefaultBalanceGains;
      event("INFO", "GAINS", "default runtime gains restored");
      return;
    }
    if (first == nullptr) {
      event("WARN", "BAD_GAINS", "four gain values are required");
      return;
    }
    char* second = strtok_r(nullptr, " ", &save);
    char* third = strtok_r(nullptr, " ", &save);
    char* fourth = strtok_r(nullptr, " ", &save);
    char* extra = strtok_r(nullptr, " ", &save);
    if (second == nullptr || third == nullptr || fourth == nullptr ||
        extra != nullptr) {
      event("WARN", "BAD_GAINS", "four gain values are required");
      return;
    }
    furuta::Gains candidate{};
    if (!parseStrictFloat(first, candidate.arm_angle) ||
        !parseStrictFloat(second, candidate.pendulum_angle) ||
        !parseStrictFloat(third, candidate.arm_velocity) ||
        !parseStrictFloat(fourth, candidate.pendulum_velocity) ||
        !gainsAreAllowed(candidate)) {
      event("WARN", "BAD_GAINS", "values or signs exceed firmware limits");
      return;
    }
    gains = candidate;
    event("INFO", "GAINS", "runtime gains accepted; they reset on reboot");
  } else if (std::strcmp(verb, "swing_settings") == 0) {
    if (mode != Mode::kDisarmed && mode != Mode::kTest) {
      event("WARN", "REFUSED", "change swing settings only while motor is disabled");
      return;
    }
    char* values[5]{};
    for (size_t index = 0; index < 5U; ++index) {
      values[index] = strtok_r(nullptr, " ", &save);
    }
    if (values[0] == nullptr || values[1] == nullptr ||
        values[2] == nullptr || values[3] == nullptr ||
        values[4] == nullptr ||
        strtok_r(nullptr, " ", &save) != nullptr) {
      event("WARN", "BAD_SWING", "energy damping centering kick torque-limit are required");
      return;
    }
    furuta::SwingSettings candidate{};
    if (!parseStrictFloat(values[0], candidate.energy_gain) ||
        !parseStrictFloat(values[1], candidate.arm_damping) ||
        !parseStrictFloat(values[2], candidate.arm_centering) ||
        !parseStrictFloat(values[3], candidate.startup_kick_nm) ||
        !parseStrictFloat(values[4], candidate.torque_limit_nm) ||
        !furuta::swingSettingsWithinLimits(
            candidate, config::kSwingSettingLimits)) {
      event("WARN", "BAD_SWING", "swing values exceed firmware limits");
      return;
    }
    swing_settings = candidate;
    event("INFO", "SWING_SETTINGS", "runtime swing settings accepted");
  } else if (std::strcmp(verb, "tune_start") == 0) {
    const char* confirmation = strtok_r(nullptr, " ", &save);
    if (mode != Mode::kDisarmed && mode != Mode::kTest) {
      event("WARN", "REFUSED", "disarm before a tuning trial");
    } else if (!config::kMechanismSetupComplete) {
      event("WARN", "REFUSED", "mechanism measurements and calculated gains are incomplete");
    } else if (confirmation == nullptr ||
               std::strcmp(confirmation, "CONFIRM") != 0) {
      event("WARN", "REFUSED", "tuning confirmation missing");
    } else if (!zero_valid || !encoder_healthy || !odrive_online ||
               !browserDeadmanFresh(millis())) {
      event("WARN", "REFUSED", "setup checks are incomplete");
    } else if (std::fabs(state.pendulum_angle_rad) >
                   config::kTuningStartAngleRad ||
               std::fabs(state.pendulum_velocity_rad_s) >
                   config::kTuningStartRateRadS) {
      event("WARN", "REFUSED", "hold pendulum nearly upright and motionless");
    } else if (!armODriveTorque()) {
      enterFault(odrive_arm_failure[0] == '\0'
                     ? "ODrive refused tuning arm request"
                     : odrive_arm_failure);
    } else {
      mode = Mode::kTuning;
      tuning_started_ms = millis();
      event("WARN", "TUNING", "low-torque Space dead-man tuning active");
    }
  } else if (std::strcmp(verb, "swing_start") == 0) {
    const char* confirmation = strtok_r(nullptr, " ", &save);
    const float down_error =
        furuta::downAngleError(state.pendulum_angle_rad);
    if (mode != Mode::kDisarmed) {
      event("WARN", "REFUSED", "disarm before a swing-up tuning trial");
    } else if (!config::kSwingTuningEnabled ||
               !config::kMechanismSetupComplete) {
      event("WARN", "REFUSED", "guarded swing-up tuning is locked");
    } else if (confirmation == nullptr ||
               std::strcmp(confirmation, "CONFIRM") != 0) {
      event("WARN", "REFUSED", "swing-up tuning confirmation missing");
    } else if (!zero_valid || !encoder_healthy || !odrive_online ||
               !browserDeadmanFresh(millis())) {
      event("WARN", "REFUSED", "setup checks are incomplete");
    } else if (down_error >
                   config::kSwingPrepositionStartDownToleranceRad ||
               std::fabs(state.pendulum_velocity_rad_s) >
                   config::kSwingPrepositionStartPendulumRateRadS ||
               std::fabs(state.arm_angle_rad) >
                   config::kSwingPrepositionStartArmAngleRad ||
               std::fabs(state.arm_velocity_rad_s) >
                   config::kSwingPrepositionStartArmRateRadS) {
      event("WARN", "REFUSED", "hold arm and hanging pendulum inside safe start envelope");
    } else if (!armODrivePosition()) {
      enterFault(odrive_arm_failure[0] == '\0'
                     ? "ODrive refused guarded swing-up request"
                     : odrive_arm_failure);
    } else {
      guarded_swing_trial = true;
      mode = Mode::kSwingCentering;
      swing_phase_started_ms = millis();
      swing_settle_ready_ms = 0U;
      event("WARN", "SWING_TUNING", "guarded trial active; slowly centering arm");
    }
  } else if (std::strcmp(verb, "arm") == 0) {
    const char* confirmation = strtok_r(nullptr, " ", &save);
    if (mode != Mode::kDisarmed) {
      event("WARN", "REFUSED", "controller must be disarmed");
    } else if (!config::kMechanismSetupComplete) {
      event("WARN", "REFUSED", "mechanism measurements and calculated gains are incomplete");
    } else if (!config::kAutomaticSwingUpEnabled) {
      event("WARN", "REFUSED", "automatic swing-up remains locked pending stable upright trials");
    } else if (confirmation == nullptr ||
               std::strcmp(confirmation, "CONFIRM") != 0) {
      event("WARN", "REFUSED", "arm confirmation missing");
    } else if (!zero_valid || !encoder_healthy || !odrive_online ||
               !browserDeadmanFresh(millis())) {
      event("WARN", "REFUSED", "setup checks are incomplete");
    } else if (!armODrivePosition()) {
      enterFault(odrive_arm_failure[0] == '\0'
                     ? "ODrive refused closed-loop control"
                     : odrive_arm_failure);
    } else {
      guarded_swing_trial = false;
      mode = Mode::kSwingCentering;
      swing_phase_started_ms = millis();
      swing_settle_ready_ms = 0U;
      event("WARN", "ARMED", "automatic centering, settling, swing-up, and balance active");
    }
  } else if (std::strcmp(verb, "status") == 0) {
    last_telemetry_ms = 0;
  } else if (std::strcmp(verb, "help") == 0) {
    event("INFO", "HELP", "zero | odrive_clear | gains a p av pv | swing_settings energy damping centering kick torque-limit | tune_start CONFIRM | swing_start CONFIRM | arm CONFIRM | disarm | status");
  } else {
    event("WARN", "UNKNOWN", original);
  }
}

void readConsole() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      if (command_length > 0U) executeCommand();
      command_length = 0;
    } else if (command_length + 1U < sizeof(command_buffer)) {
      command_buffer[command_length++] = incoming;
    } else {
      command_length = 0;
      event("WARN", "COMMAND", "command too long");
    }
  }
}

void printTelemetry() {
  Serial.print(F("@T,"));
  Serial.print(millis());
  Serial.print(','); Serial.print(modeName());
  Serial.print(','); Serial.print(state.arm_angle_rad, 5);
  Serial.print(','); Serial.print(state.pendulum_angle_rad, 5);
  Serial.print(','); Serial.print(state.arm_velocity_rad_s, 4);
  Serial.print(','); Serial.print(state.pendulum_velocity_rad_s, 4);
  Serial.print(','); Serial.print(commanded_torque_nm, 5);
  Serial.print(','); Serial.print(browserDeadmanFresh(millis()) ? 1 : 0);
  Serial.print(','); Serial.print(odrive_online ? 1 : 0);
  Serial.print(','); Serial.print(encoderStatus());
  Serial.print(','); Serial.print(zero_valid ? 1 : 0);
  Serial.print(','); Serial.print(pendulum_count);
  Serial.print(','); Serial.print(encoder_diagnostics.agc);
  Serial.print(','); Serial.print(encoder.parityErrors());
  Serial.print(','); Serial.print(encoder.protocolErrors());
  Serial.print(','); Serial.print(gains.arm_angle, 4);
  Serial.print(','); Serial.print(gains.pendulum_angle, 4);
  Serial.print(','); Serial.print(gains.arm_velocity, 4);
  Serial.print(','); Serial.print(gains.pendulum_velocity, 4);
  Serial.print(','); Serial.print(odrive_active_errors);
  Serial.print(','); Serial.print(last_tick_duration_us);
  Serial.print(','); Serial.print(maximum_tick_duration_us);
  Serial.print(','); Serial.print(config::kMechanismSetupComplete ? 1 : 0);
  Serial.print(','); Serial.print(config::kAutomaticSwingUpEnabled ? 1 : 0);
  Serial.print(','); Serial.print(config::kSwingTuningEnabled ? 1 : 0);
  Serial.print(','); Serial.print(guarded_swing_trial ? 1 : 0);
  Serial.print(','); Serial.print(swing_settings.energy_gain, 4);
  Serial.print(','); Serial.print(swing_settings.arm_damping, 4);
  Serial.print(','); Serial.print(swing_settings.arm_centering, 4);
  Serial.print(','); Serial.print(swing_settings.startup_kick_nm, 4);
  Serial.print(','); Serial.print(swing_settings.torque_limit_nm, 4);
  Serial.print(','); Serial.println(fault_reason[0] == '\0' ? "-" : fault_reason);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  encoder.begin();
  odrive.begin();
  delay(20);
  requestIdle();
  next_control_us = micros() + config::kControlPeriodUs;
  Serial.println(F("@HELLO,7,Teensy 4.1,ODrive UART,AS5048A SPI"));
  event("INFO", "READY", "controller booted DISARMED; open the setup page");
}

void loop() {
  readConsole();

  const uint32_t now_us = micros();
  if (static_cast<int32_t>(now_us - next_control_us) >= 0) {
    next_control_us += config::kControlPeriodUs;
    if (static_cast<int32_t>(now_us - next_control_us) >= 0) {
      ++consecutive_deadline_misses;
      next_control_us = now_us + config::kControlPeriodUs;
      if (isActive() && consecutive_deadline_misses >=
                            config::kMaximumConsecutiveDeadlineMisses) {
        enterFault("repeated control deadline misses");
      }
    } else {
      consecutive_deadline_misses = 0;
    }
    const uint32_t tick_started_us = micros();
    runControlTick();
    last_tick_duration_us = micros() - tick_started_us;
    if (last_tick_duration_us > maximum_tick_duration_us) {
      maximum_tick_duration_us = last_tick_duration_us;
    }
  }

  const uint32_t now_ms = millis();
  const uint32_t telemetry_period_ms =
      mode == Mode::kTuning || guarded_swing_trial
          ? config::kTuningTelemetryPeriodMs
          : config::kTelemetryPeriodMs;
  if (now_ms - last_telemetry_ms >= telemetry_period_ms) {
    last_telemetry_ms = now_ms;
    printTelemetry();
  }
}
