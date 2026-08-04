#pragma once

#include <cstdint>

namespace as5048a {

constexpr uint16_t kCountsPerRevolution = 1U << 14;
constexpr uint16_t kDataMask = kCountsPerRevolution - 1U;
constexpr uint16_t kReadFlag = 1U << 14;
constexpr uint16_t kErrorFlag = 1U << 14;
constexpr uint16_t kParityFlag = 1U << 15;
constexpr uint16_t kRegisterClearError = 0x0001U;
constexpr uint16_t kRegisterDiagnostics = 0x3FFCU;
constexpr uint16_t kRegisterAngle = 0x3FFFU;
constexpr uint16_t kNopCommand = 0x0000U;

constexpr bool hasOddParity(uint16_t value) {
  bool odd = false;
  while (value != 0U) {
    odd = !odd;
    value &= static_cast<uint16_t>(value - 1U);
  }
  return odd;
}

constexpr uint16_t withEvenParity(uint16_t value) {
  value &= 0x7FFFU;
  return hasOddParity(value) ? static_cast<uint16_t>(value | kParityFlag)
                             : value;
}

constexpr uint16_t makeReadCommand(const uint16_t address) {
  return withEvenParity(
      static_cast<uint16_t>(kReadFlag | (address & kDataMask)));
}

struct Response {
  bool parity_ok;
  bool error_flag;
  uint16_t data;
};

constexpr Response parseResponse(const uint16_t frame) {
  return {!hasOddParity(frame), (frame & kErrorFlag) != 0U,
          static_cast<uint16_t>(frame & kDataMask)};
}

struct Diagnostics {
  uint8_t agc;
  bool offset_compensation_finished;
  bool cordic_overflow;
  bool magnetic_field_too_weak;
  bool magnetic_field_too_strong;

  constexpr bool healthy() const {
    return offset_compensation_finished && !cordic_overflow &&
           !magnetic_field_too_weak && !magnetic_field_too_strong;
  }
};

constexpr Diagnostics decodeDiagnostics(const uint16_t value) {
  return {static_cast<uint8_t>(value & 0x00FFU), (value & (1U << 8)) != 0U,
          (value & (1U << 9)) != 0U, (value & (1U << 10)) != 0U,
          (value & (1U << 11)) != 0U};
}

}  // namespace as5048a
