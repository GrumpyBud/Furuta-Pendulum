#include <unity.h>

#include "as5048a_protocol.hpp"

void test_angle_read_command_has_even_parity() {
  TEST_ASSERT_EQUAL_HEX16(
      0xFFFFU, as5048a::makeReadCommand(as5048a::kRegisterAngle));
}

void test_diagnostics_register_address_matches_datasheet() {
  TEST_ASSERT_EQUAL_HEX16(0x3FFCU, as5048a::kRegisterDiagnostics);
}

void test_valid_response_returns_all_fourteen_data_bits() {
  const uint16_t frame = as5048a::withEvenParity(0x2A55U);
  const auto response = as5048a::parseResponse(frame);
  TEST_ASSERT_TRUE(response.parity_ok);
  TEST_ASSERT_FALSE(response.error_flag);
  TEST_ASSERT_EQUAL_HEX16(0x2A55U, response.data);
}

void test_bad_parity_is_detected() {
  const uint16_t frame =
      static_cast<uint16_t>(as5048a::withEvenParity(0x1234U) ^ 0x0001U);
  TEST_ASSERT_FALSE(as5048a::parseResponse(frame).parity_ok);
}

void test_protocol_error_flag_is_detected() {
  const uint16_t frame = as5048a::withEvenParity(
      static_cast<uint16_t>(as5048a::kErrorFlag | 0x0042U));
  const auto response = as5048a::parseResponse(frame);
  TEST_ASSERT_TRUE(response.parity_ok);
  TEST_ASSERT_TRUE(response.error_flag);
}

void test_diagnostics_decode_healthy_and_weak_magnet() {
  const auto healthy = as5048a::decodeDiagnostics(1U << 8);
  TEST_ASSERT_TRUE(healthy.healthy());
  const auto weak = as5048a::decodeDiagnostics((1U << 8) | (1U << 10));
  TEST_ASSERT_FALSE(weak.healthy());
  TEST_ASSERT_TRUE(weak.magnetic_field_too_weak);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_angle_read_command_has_even_parity);
  RUN_TEST(test_diagnostics_register_address_matches_datasheet);
  RUN_TEST(test_valid_response_returns_all_fourteen_data_bits);
  RUN_TEST(test_bad_parity_is_detected);
  RUN_TEST(test_protocol_error_flag_is_detected);
  RUN_TEST(test_diagnostics_decode_healthy_and_weak_magnet);
  return UNITY_END();
}
