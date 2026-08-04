#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "as5048a_protocol.hpp"
#include "control_math.hpp"
#include "uart_protocol.hpp"

void test_wrap_angle_range_and_boundaries() {
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, -furuta::kPi,
                           furuta::wrapAngle(furuta::kPi));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 0.2F,
                           furuta::wrapAngle(furuta::kTwoPi + 0.2F));
  TEST_ASSERT_TRUE(furuta::wrapAngle(-100.0F) >= -furuta::kPi);
  TEST_ASSERT_TRUE(furuta::wrapAngle(100.0F) < furuta::kPi);
}

void test_balance_torque_uses_documented_state_order() {
  const furuta::State state{1.0F, 2.0F, 3.0F, 4.0F};
  const furuta::Gains gains{1.0F, 10.0F, 100.0F, 1000.0F};
  TEST_ASSERT_FLOAT_WITHIN(1.0e-5F, -4321.0F,
                           furuta::balanceTorque(state, gains));
}

void test_slew_limit_bounds_both_directions() {
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 0.1F,
                           furuta::slewLimit(0.0F, 1.0F, 0.1F));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, -0.1F,
                           furuta::slewLimit(0.0F, -1.0F, 0.1F));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 0.04F,
                           furuta::slewLimit(0.0F, 0.04F, 0.1F));
}

void test_as5048a_command_and_response_parity() {
  const uint16_t command =
      as5048a::makeReadCommand(as5048a::kRegisterAngle);
  TEST_ASSERT_FALSE(as5048a::hasOddParity(command));
  const auto response =
      as5048a::parseResponse(as5048a::withEvenParity(0x1234U));
  TEST_ASSERT_TRUE(response.parity_ok);
  TEST_ASSERT_FALSE(response.error_flag);
  TEST_ASSERT_EQUAL_UINT16(0x1234U, response.data);
}

void test_ascii_checksum_matches_odrive_documentation() {
  const char example[] = "r vbus_voltage ";
  TEST_ASSERT_EQUAL_UINT8(93U,
                          odrive_ascii::checksum(example, sizeof(example) - 1U));
}

void test_ascii_response_validation_and_feedback_parse() {
  char line[40] = "1.250000 -2.500000 *62";
  const size_t star_offset = static_cast<size_t>(std::strchr(line, '*') - line);
  const uint8_t correct = odrive_ascii::checksum(line, star_offset);
  std::snprintf(std::strchr(line, '*') + 1, 4, "%u",
                static_cast<unsigned>(correct));
  TEST_ASSERT_TRUE(odrive_ascii::validateAndStripChecksum(line));
  float position = 0.0F;
  float velocity = 0.0F;
  TEST_ASSERT_TRUE(odrive_ascii::parseFeedback(line, position, velocity));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 1.25F, position);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, -2.5F, velocity);
}

void test_ascii_checksum_rejects_corruption() {
  char line[] = "0.0 0.0 *1";
  TEST_ASSERT_FALSE(odrive_ascii::validateAndStripChecksum(line));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_wrap_angle_range_and_boundaries);
  RUN_TEST(test_balance_torque_uses_documented_state_order);
  RUN_TEST(test_slew_limit_bounds_both_directions);
  RUN_TEST(test_as5048a_command_and_response_parity);
  RUN_TEST(test_ascii_checksum_matches_odrive_documentation);
  RUN_TEST(test_ascii_response_validation_and_feedback_parse);
  RUN_TEST(test_ascii_checksum_rejects_corruption);
  return UNITY_END();
}
