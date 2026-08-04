#include <Arduino.h>
#include <SPI.h>

#include <cstring>

#include "as5048a_protocol.hpp"

namespace {

constexpr uint8_t kChipSelectPin = 10;
constexpr uint32_t kSpiClockHz = 1000000;
constexpr uint32_t kSamplePeriodUs = 10000;  // 100 samples/second
const SPISettings kSpiSettings(kSpiClockHz, MSBFIRST, SPI_MODE1);

uint32_t next_sample_us = 0;
uint32_t parity_error_count = 0;
uint32_t protocol_error_count = 0;
uint16_t zero_offset_count = 0;
char command_buffer[24] = {};
uint8_t command_length = 0;

uint16_t transferFrame(const uint16_t transmit) {
  SPI.beginTransaction(kSpiSettings);
  digitalWrite(kChipSelectPin, LOW);
  delayNanoseconds(400);  // AS5048A requires at least 350 ns before clocking.
  const uint16_t received = SPI.transfer16(transmit);
  delayNanoseconds(100);  // Exceeds the 50 ns final-clock-to-CS requirement.
  digitalWrite(kChipSelectPin, HIGH);
  SPI.endTransaction();
  delayNanoseconds(400);  // AS5048A requires at least 350 ns CS high.
  return received;
}

void clearEncoderError() {
  transferFrame(as5048a::makeReadCommand(as5048a::kRegisterClearError));
  transferFrame(as5048a::kNopCommand);
}

bool readRegister(const uint16_t address, uint16_t& value) {
  // AS5048A reads are pipelined: the requested data is returned during the
  // following 16-bit transaction.
  transferFrame(as5048a::makeReadCommand(address));
  const auto response = as5048a::parseResponse(
      transferFrame(as5048a::kNopCommand));
  if (!response.parity_ok) {
    ++parity_error_count;
    return false;
  }
  if (response.error_flag) {
    ++protocol_error_count;
    clearEncoderError();
    return false;
  }
  value = response.data;
  return true;
}

const __FlashStringHelper* diagnosticStatus(
    const as5048a::Diagnostics& diagnostics) {
  if (!diagnostics.offset_compensation_finished) return F("OCF_NOT_READY");
  if (diagnostics.cordic_overflow) return F("CORDIC_OVERFLOW");
  if (diagnostics.magnetic_field_too_weak) return F("MAGNET_TOO_WEAK");
  if (diagnostics.magnetic_field_too_strong) return F("MAGNET_TOO_STRONG");
  return F("OK");
}

void zeroEncoder() {
  uint16_t current_count = 0;
  if (!readRegister(as5048a::kRegisterAngle, current_count)) {
    Serial.println(F("# ZERO_FAILED,ENCODER_READ_ERROR"));
    return;
  }
  zero_offset_count = current_count;
  Serial.print(F("# ZEROED,"));
  Serial.println(zero_offset_count);
}

void runCommand() {
  command_buffer[command_length] = '\0';
  for (uint8_t index = 0; index < command_length; ++index) {
    if (command_buffer[index] >= 'A' && command_buffer[index] <= 'Z') {
      command_buffer[index] += 'a' - 'A';
    }
  }

  if (strcmp(command_buffer, "zero") == 0) {
    zeroEncoder();
  } else if (strcmp(command_buffer, "help") == 0) {
    Serial.println(F("# COMMANDS,zero,help"));
  } else if (command_length > 0) {
    Serial.print(F("# UNKNOWN_COMMAND,"));
    Serial.println(command_buffer);
  }
  command_length = 0;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      if (command_length > 0) runCommand();
      continue;
    }
    if (command_length < sizeof(command_buffer) - 1U) {
      command_buffer[command_length++] = incoming;
    } else {
      command_length = 0;
      Serial.println(F("# COMMAND_TOO_LONG"));
    }
  }
}

void printSample(const uint32_t now_us) {
  uint16_t absolute_count = 0;
  uint16_t diagnostic_value = 0;
  if (!readRegister(as5048a::kRegisterAngle, absolute_count) ||
      !readRegister(as5048a::kRegisterDiagnostics, diagnostic_value)) {
    Serial.print(now_us);
    Serial.print(F(",ERR,,,,READ_ERROR,"));
    Serial.print(parity_error_count);
    Serial.print(',');
    Serial.println(protocol_error_count);
    return;
  }

  const auto diagnostics = as5048a::decodeDiagnostics(diagnostic_value);
  const uint16_t zeroed_count =
      as5048a::relativeCount(absolute_count, zero_offset_count);
  const float angle_degrees =
      zeroed_count * 360.0F / as5048a::kCountsPerRevolution;

  Serial.print(now_us);
  Serial.print(',');
  Serial.print(absolute_count);
  Serial.print(',');
  Serial.print(zeroed_count);
  Serial.print(',');
  Serial.print(angle_degrees, 4);
  Serial.print(',');
  Serial.print(diagnostics.agc);
  Serial.print(',');
  Serial.print(diagnosticStatus(diagnostics));
  Serial.print(',');
  Serial.print(parity_error_count);
  Serial.print(',');
  Serial.println(protocol_error_count);
}

}  // namespace

void setup() {
  pinMode(kChipSelectPin, OUTPUT);
  digitalWrite(kChipSelectPin, HIGH);
  SPI.begin();
  Serial.begin(115200);
  const uint32_t serial_wait_start_ms = millis();
  while (!Serial && millis() - serial_wait_start_ms < 3000U) {
  }
  delay(20);  // AS5048A startup is specified at up to 10 ms.
  clearEncoderError();
  Serial.println(F("time_us,absolute_count,zeroed_count,angle_deg,agc,status,parity_errors,protocol_errors"));
  Serial.println(F("# READY,send zero to set the current position to 0 degrees"));
  next_sample_us = micros();
}

void loop() {
  readSerialCommands();
  const uint32_t now_us = micros();
  if (static_cast<int32_t>(now_us - next_sample_us) < 0) return;
  next_sample_us += kSamplePeriodUs;
  if (static_cast<int32_t>(now_us - next_sample_us) >= 0) {
    next_sample_us = now_us + kSamplePeriodUs;
  }
  printSample(now_us);
}
