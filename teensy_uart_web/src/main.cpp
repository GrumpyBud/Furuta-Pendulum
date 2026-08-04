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
  kSwingUp,
  kBalance,
  kTuning,
  kFault,
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

  bool request(const char* command, char* response, const size_t response_size) {
    drainInput();
    if (!send(command)) return false;
    size_t length = 0;
    const uint32_t started_us = micros();
    while (micros() - started_us < config::kUartResponseTimeoutUs) {
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

  bool readUnsigned(const char* property, uint32_t& value) {
    char command[80]{};
    const int written =
        std::snprintf(command, sizeof(command), "r %s", property);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    char response[48]{};
    if (!request(command, response, sizeof(response))) return false;
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(response, &end, 0);
    while (end != nullptr && *end == ' ') ++end;
    if (end == response || end == nullptr || *end != '\0') return false;
    value = static_cast<uint32_t>(parsed);
    return true;
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

  bool writeProperty(const char* property, const char* value) {
    char command[112]{};
    const int written = std::snprintf(command, sizeof(command), "w %s %s",
                                      property, value);
    if (written <= 0 || static_cast<size_t>(written) >= sizeof(command)) {
      return false;
    }
    return send(command);
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
uint32_t tuning_keepalive_ms = 0;
uint32_t run_started_ms = 0;
uint32_t run_keepalive_ms = 0;
uint32_t last_state_sample_us = 0;
uint32_t consecutive_encoder_errors = 0;
uint32_t consecutive_deadline_misses = 0;
uint32_t last_tick_duration_us = 0;
uint32_t maximum_tick_duration_us = 0;
uint32_t odrive_active_errors = 0;
bool zero_valid = false;
bool odrive_online = false;
bool encoder_healthy = false;
char fault_reason[64]{};
char command_buffer[128]{};
size_t command_length = 0;

bool isActive() {
  return mode == Mode::kSwingUp || mode == Mode::kBalance ||
         mode == Mode::kTuning;
}

const char* modeName() {
  switch (mode) {
    case Mode::kDisarmed: return "DISARMED";
    case Mode::kTest: return "TEST";
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
  commanded_torque_nm = 0.0F;
}

void enterFault(const char* reason) {
  if (mode == Mode::kFault) return;
  requestIdle();
  mode = Mode::kFault;
  zero_valid = false;
  std::snprintf(fault_reason, sizeof(fault_reason), "%s", reason);
  event("ERROR", "FAULT", fault_reason);
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
  if (digitalRead(config::kEstopPin) != LOW) {
    enterFault("E-stop loop open");
  } else if (!encoder_healthy ||
             consecutive_encoder_errors >=
                 config::kMaximumConsecutiveEncoderErrors) {
    enterFault("AS5048A encoder unhealthy");
  } else if (!odrive_online ||
             now_us - last_feedback_us > config::kFeedbackTimeoutUs) {
    enterFault("ODrive UART feedback timeout");
  } else if (odrive_active_errors != 0U) {
    enterFault("ODrive reports an active error");
  } else if (!std::isfinite(state.arm_angle_rad) ||
             !std::isfinite(state.pendulum_angle_rad) ||
             !std::isfinite(state.arm_velocity_rad_s) ||
             !std::isfinite(state.pendulum_velocity_rad_s)) {
    enterFault("non-finite sensor value");
  } else if (std::fabs(state.arm_angle_rad) > config::kArmAngleLimitRad) {
    enterFault("arm travel limit");
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
    odrive_online = false;
    if (isActive()) enterFault("ODrive health query failed");
    return;
  }
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
    if (!readODriveFeedback()) enterFault("ODrive UART feedback failed");
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
      enterFault("AS5048A repeated read errors");
    }
    return;
  }

  if (!isActive() || !stateIsHealthy(micros())) return;

  float requested_torque = 0.0F;
  if (mode == Mode::kTuning) {
    if (now_ms - tuning_keepalive_ms > config::kTuningKeepaliveTimeoutMs) {
      enterFault("tuning hold released or browser disconnected");
      return;
    }
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
  } else {
    if (now_ms - run_keepalive_ms > config::kRunKeepaliveTimeoutMs) {
      enterFault("run control page disconnected or stopped responding");
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
      if (startup_elapsed_ms < 2U * config::kSwingStartupKickPhaseMs) {
        requested_torque =
            startup_elapsed_ms < config::kSwingStartupKickPhaseMs
                ? config::kSwingStartupKickNm
                : -config::kSwingStartupKickNm;
      } else {
        requested_torque = furuta::swingUpTorque(
            state, config::kPendulumMassKg, config::kPendulumComLengthM,
            config::kPendulumInertiaKgM2, config::kSwingEnergyGain,
            config::kSwingArmDamping);
      }
      requested_torque -= config::kSwingArmCentering * state.arm_angle_rad;
      requested_torque = furuta::clamp(
          requested_torque, -config::kSwingTorqueLimitNm,
          config::kSwingTorqueLimitNm);
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
    enterFault("ODrive UART torque write failed");
  }
}

bool gainsAreAllowed(const furuta::Gains& candidate) {
  return std::isfinite(candidate.arm_angle) &&
         std::isfinite(candidate.pendulum_angle) &&
         std::isfinite(candidate.arm_velocity) &&
         std::isfinite(candidate.pendulum_velocity) &&
         candidate.arm_angle >= 0.0F && candidate.pendulum_angle <= 0.0F &&
         candidate.arm_velocity >= 0.0F &&
         candidate.pendulum_velocity <= 0.0F &&
         candidate.arm_angle <= config::kGainAbsoluteLimits.arm_angle &&
         -candidate.pendulum_angle <=
             config::kGainAbsoluteLimits.pendulum_angle &&
         candidate.arm_velocity <=
             config::kGainAbsoluteLimits.arm_velocity &&
         -candidate.pendulum_velocity <=
             config::kGainAbsoluteLimits.pendulum_velocity;
}

bool parseStrictFloat(const char* text, float& value) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0' &&
         std::isfinite(value);
}

bool armODrive() {
  odrive.drainInput();
  if (!odrive.clearErrors() ||
      !odrive.writeProperty("axis0.config.watchdog_timeout",
                            config::kODriveWatchdogSeconds) ||
      !odrive.writeProperty("axis0.config.enable_watchdog", "1") ||
      !odrive.writeProperty("axis0.controller.config.input_mode", "1") ||
      !odrive.writeProperty("axis0.controller.config.control_mode", "1") ||
      !odrive.setTorque(0.0F) ||
      !odrive.writeProperty("axis0.requested_state", "8")) {
    return false;
  }
  delay(20);
  uint32_t current_state = 0;
  uint32_t active_errors = 0;
  if (!odrive.readUnsigned("axis0.current_state", current_state) ||
      !odrive.readUnsigned("axis0.active_errors", active_errors)) {
    return false;
  }
  odrive_active_errors = active_errors;
  commanded_torque_nm = 0.0F;
  return current_state == 8U && active_errors == 0U && readODriveFeedback();
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

void executeCommand() {
  command_buffer[command_length] = '\0';
  char original[sizeof(command_buffer)]{};
  std::snprintf(original, sizeof(original), "%s", command_buffer);
  char* save = nullptr;
  char* verb = strtok_r(command_buffer, " ", &save);
  if (verb == nullptr) return;

  if (std::strcmp(verb, "disarm") == 0) {
    disarm(true);
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
  } else if (std::strcmp(verb, "tune_start") == 0) {
    const char* confirmation = strtok_r(nullptr, " ", &save);
    if (mode != Mode::kDisarmed && mode != Mode::kTest) {
      event("WARN", "REFUSED", "disarm before a tuning trial");
    } else if (confirmation == nullptr ||
               std::strcmp(confirmation, "CONFIRM") != 0) {
      event("WARN", "REFUSED", "tuning confirmation missing");
    } else if (!zero_valid || digitalRead(config::kEstopPin) != LOW ||
               !encoder_healthy || !odrive_online) {
      event("WARN", "REFUSED", "setup checks are incomplete");
    } else if (std::fabs(state.pendulum_angle_rad) >
                   config::kTuningStartAngleRad ||
               std::fabs(state.pendulum_velocity_rad_s) >
                   config::kTuningStartRateRadS) {
      event("WARN", "REFUSED", "hold pendulum nearly upright and motionless");
    } else if (!armODrive()) {
      enterFault("ODrive refused tuning arm request");
    } else {
      mode = Mode::kTuning;
      tuning_started_ms = millis();
      tuning_keepalive_ms = tuning_started_ms;
      event("WARN", "TUNING", "low-torque dead-man tuning active");
    }
  } else if (std::strcmp(verb, "tune_keepalive") == 0) {
    if (mode == Mode::kTuning) tuning_keepalive_ms = millis();
  } else if (std::strcmp(verb, "run_keepalive") == 0) {
    if (mode == Mode::kSwingUp || mode == Mode::kBalance) {
      run_keepalive_ms = millis();
    }
  } else if (std::strcmp(verb, "arm") == 0) {
    const char* confirmation = strtok_r(nullptr, " ", &save);
    if (mode != Mode::kDisarmed) {
      event("WARN", "REFUSED", "controller must be disarmed");
    } else if (confirmation == nullptr ||
               std::strcmp(confirmation, "CONFIRM") != 0) {
      event("WARN", "REFUSED", "arm confirmation missing");
    } else if (!zero_valid || digitalRead(config::kEstopPin) != LOW ||
               !encoder_healthy || !odrive_online) {
      event("WARN", "REFUSED", "setup checks are incomplete");
    } else if (!armODrive()) {
      enterFault("ODrive refused closed-loop control");
    } else {
      mode = Mode::kSwingUp;
      run_started_ms = millis();
      run_keepalive_ms = run_started_ms;
      event("WARN", "ARMED", "automatic swing-up and balance active");
    }
  } else if (std::strcmp(verb, "status") == 0) {
    last_telemetry_ms = 0;
  } else if (std::strcmp(verb, "help") == 0) {
    event("INFO", "HELP", "zero | test_start | test_stop | gains a p av pv | tune_start CONFIRM | tune_keepalive | arm CONFIRM | run_keepalive | disarm | status");
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
  Serial.print(','); Serial.print(digitalRead(config::kEstopPin) == LOW ? 1 : 0);
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
  Serial.print(','); Serial.println(fault_reason[0] == '\0' ? "-" : fault_reason);
}

}  // namespace

void setup() {
  pinMode(config::kEstopPin, INPUT_PULLUP);
  Serial.begin(115200);
  encoder.begin();
  odrive.begin();
  delay(20);
  requestIdle();
  next_control_us = micros() + config::kControlPeriodUs;
  Serial.println(F("@HELLO,2,Teensy 4.1,ODrive UART,AS5048A SPI"));
  event("INFO", "READY", "controller booted DISARMED; open the setup page");
}

void loop() {
  readConsole();
  if (isActive() && digitalRead(config::kEstopPin) != LOW) {
    enterFault("E-stop loop open");
  }

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
  if (now_ms - last_telemetry_ms >= config::kTelemetryPeriodMs) {
    last_telemetry_ms = now_ms;
    printTelemetry();
  }
}
