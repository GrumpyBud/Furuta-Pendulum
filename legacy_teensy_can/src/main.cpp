// Legacy Teensy 4.1/CAN implementation.
#include <Arduino.h>
#include <Encoder.h>
#include <ODriveCAN.h>
#include <FlexCAN_T4.h>
#include <ODriveFlexCAN.hpp>

#include <cmath>

#include "config.hpp"
#include "control_math.hpp"

struct ODriveStatus;  // Workaround for a name collision in Teensy's core.

namespace {

enum class Mode : uint8_t { kDisarmed, kSwingUp, kBalance, kFault };

struct DriveData {
  Heartbeat_msg_t heartbeat{};
  Get_Encoder_Estimates_msg_t feedback{};
  volatile uint32_t heartbeat_us = 0;
  volatile uint32_t feedback_us = 0;
};

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_bus;
ODriveCAN odrive(wrap_can_intf(can_bus), config::kODriveNodeId);
Encoder pendulum_encoder(config::kPendulumEncoderA, config::kPendulumEncoderB);
DriveData drive;
Mode mode = Mode::kDisarmed;
int32_t pendulum_down_count = 0;
float arm_zero_turns = 0.0F;
float last_pendulum_angle = 0.0F;
float pendulum_velocity = 0.0F;
uint32_t next_control_us = 0;
uint32_t last_telemetry_ms = 0;
char command[24]{};
size_t command_length = 0;
bool zero_is_valid = false;

void enterFault(const __FlashStringHelper* reason) {
  if (mode == Mode::kFault) return;
  odrive.setTorque(0.0F);
  odrive.setState(AXIS_STATE_IDLE);
  mode = Mode::kFault;
  Serial.print(F("FAULT: "));
  Serial.println(reason);
}

void onHeartbeat(Heartbeat_msg_t& message, void*) {
  drive.heartbeat = message;
  drive.heartbeat_us = micros();
}

void onFeedback(Get_Encoder_Estimates_msg_t& message, void*) {
  drive.feedback = message;
  drive.feedback_us = micros();
}

void onCanMessage(const CAN_message_t& message) {
  onReceive(message, odrive);
}

const __FlashStringHelper* modeName() {
  switch (mode) {
    case Mode::kDisarmed: return F("DISARMED");
    case Mode::kSwingUp: return F("SWING_UP");
    case Mode::kBalance: return F("BALANCE");
    case Mode::kFault: return F("FAULT");
  }
  return F("UNKNOWN");
}

furuta::State sampleState(const float dt_s) {
  const int32_t relative_count = pendulum_encoder.read() - pendulum_down_count;
  const float count_to_rad = config::kPendulumDirection * furuta::kTwoPi /
                             config::kPendulumCountsPerRevolution;
  // The encoder is zeroed hanging down, hence subtracting pi makes upright zero.
  const float pendulum_angle =
      furuta::wrapAngle(relative_count * count_to_rad - furuta::kPi);
  const float raw_velocity =
      furuta::wrapAngle(pendulum_angle - last_pendulum_angle) / dt_s;
  pendulum_velocity = furuta::lowPass(pendulum_velocity, raw_velocity,
                                      config::kVelocityFilterHz, dt_s);
  last_pendulum_angle = pendulum_angle;

  const auto feedback = drive.feedback;
  return {(feedback.Pos_Estimate - arm_zero_turns) *
              config::kMotorTurnsToArmRadians * config::kMotorDirection,
          pendulum_angle,
          feedback.Vel_Estimate * config::kMotorTurnsToArmRadians *
              config::kMotorDirection,
          pendulum_velocity};
}

bool healthy(const furuta::State& x, const uint32_t now_us) {
  if (digitalRead(config::kEstopPin) != LOW) {
    enterFault(F("E-stop open"));
  } else if (now_us - drive.heartbeat_us > config::kHeartbeatTimeoutUs) {
    enterFault(F("ODrive heartbeat timeout"));
  } else if (now_us - drive.feedback_us > config::kFeedbackTimeoutUs) {
    enterFault(F("ODrive encoder timeout"));
  } else if (drive.heartbeat.Axis_Error != 0) {
    enterFault(F("ODrive axis error"));
  } else if (!std::isfinite(x.arm_angle_rad) ||
             !std::isfinite(x.pendulum_angle_rad)) {
    enterFault(F("non-finite sensor value"));
  } else if (std::fabs(x.arm_angle_rad) > config::kArmAngleLimitRad) {
    enterFault(F("arm travel limit"));
  } else if (std::fabs(x.arm_velocity_rad_s) > config::kArmVelocityLimitRadS ||
             std::fabs(x.pendulum_velocity_rad_s) >
                 config::kPendulumVelocityLimitRadS) {
    enterFault(F("overspeed"));
  }
  return mode != Mode::kFault;
}

void runController(const uint32_t now_us) {
  constexpr float dt_s = config::kControlPeriodUs * 1.0e-6F;
  const furuta::State x = sampleState(dt_s);
  if (mode == Mode::kDisarmed || mode == Mode::kFault || !healthy(x, now_us)) {
    return;
  }

  if (mode == Mode::kSwingUp &&
      std::fabs(x.pendulum_angle_rad) < config::kCatchAngleRad &&
      std::fabs(x.pendulum_velocity_rad_s) <
          config::kCatchPendulumVelocityRadS) {
    mode = Mode::kBalance;
  } else if (mode == Mode::kBalance &&
             std::fabs(x.pendulum_angle_rad) > config::kDropAngleRad) {
    mode = Mode::kSwingUp;
  }

  float torque = 0.0F;
  if (mode == Mode::kBalance) {
    torque = furuta::balanceTorque(x, config::kBalanceGains);
  } else {
    torque = furuta::swingUpTorque(
        x, config::kPendulumMassKg, config::kPendulumComLengthM,
        config::kPendulumInertiaKgM2, config::kSwingEnergyGain,
        config::kSwingArmDamping);
    torque = furuta::clamp(torque, -config::kSwingTorqueLimitNm,
                           config::kSwingTorqueLimitNm);
  }
  torque = furuta::clamp(torque, -config::kTorqueLimitNm,
                         config::kTorqueLimitNm);
  if (!odrive.setTorque(torque)) enterFault(F("CAN transmit failed"));
}

void printStatus() {
  const furuta::State x = sampleState(config::kControlPeriodUs * 1.0e-6F);
  Serial.print(F("mode=")); Serial.print(modeName());
  Serial.print(F(" arm=")); Serial.print(x.arm_angle_rad, 4);
  Serial.print(F(" pend=")); Serial.print(x.pendulum_angle_rad, 4);
  Serial.print(F(" arm_rate=")); Serial.print(x.arm_velocity_rad_s, 3);
  Serial.print(F(" pend_rate=")); Serial.println(x.pendulum_velocity_rad_s, 3);
}

void executeCommand() {
  if (strcmp(command, "zero") == 0 && mode == Mode::kDisarmed) {
    pendulum_down_count = pendulum_encoder.read();
    arm_zero_turns = drive.feedback.Pos_Estimate;
    last_pendulum_angle = -furuta::kPi;
    pendulum_velocity = 0.0F;
    zero_is_valid = true;
    Serial.println(F("Zero saved: pendulum must be hanging straight down."));
  } else if (strcmp(command, "arm") == 0 && mode == Mode::kDisarmed) {
    const uint32_t now = micros();
    if (!zero_is_valid) {
      Serial.println(F("Refused: send zero with pendulum hanging down first."));
    } else if (digitalRead(config::kEstopPin) != LOW ||
        now - drive.heartbeat_us > config::kHeartbeatTimeoutUs ||
        now - drive.feedback_us > config::kFeedbackTimeoutUs) {
      Serial.println(F("Refused: close E-stop loop and check ODrive traffic."));
    } else {
      odrive.clearErrors();
      odrive.setControllerMode(CONTROL_MODE_TORQUE_CONTROL,
                               INPUT_MODE_PASSTHROUGH);
      odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      mode = Mode::kSwingUp;
      Serial.println(F("ARMED. Keep clear; type disarm to stop."));
    }
  } else if (strcmp(command, "disarm") == 0) {
    odrive.setTorque(0.0F);
    odrive.setState(AXIS_STATE_IDLE);
    mode = Mode::kDisarmed;
    Serial.println(F("Disarmed."));
  } else if (strcmp(command, "status") == 0) {
    printStatus();
  } else if (strcmp(command, "help") == 0) {
    Serial.println(F("Commands: zero, arm, disarm, status, help"));
  } else {
    Serial.println(F("Unknown/invalid command. Type help."));
  }
}

void readConsole() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      command[command_length] = '\0';
      if (command_length != 0) executeCommand();
      command_length = 0;
    } else if (command_length + 1 < sizeof(command)) {
      command[command_length++] = c;
    }
  }
}

}  // namespace

void setup() {
  pinMode(config::kEstopPin, INPUT_PULLUP);
  Serial.begin(115200);
  can_bus.begin();
  can_bus.setBaudRate(config::kCanBaud);
  can_bus.setMaxMB(16);
  can_bus.enableFIFO();
  can_bus.enableFIFOInterrupt();
  can_bus.onReceive(onCanMessage);
  odrive.onStatus(onHeartbeat);
  odrive.onFeedback(onFeedback);
  next_control_us = micros() + config::kControlPeriodUs;
  Serial.println(F("Furuta controller ready. Type help; start DISARMED."));
}

void loop() {
  can_bus.events();
  readConsole();
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - next_control_us) >= 0) {
    // Advance by a fixed period instead of now+period to avoid long-term drift.
    next_control_us += config::kControlPeriodUs;
    if (static_cast<int32_t>(now - next_control_us) >= 0) {
      enterFault(F("control deadline missed"));
      next_control_us = now + config::kControlPeriodUs;
    }
    runController(now);
  }
  if (millis() - last_telemetry_ms >= 1000) {
    last_telemetry_ms = millis();
    printStatus();
  }
}
