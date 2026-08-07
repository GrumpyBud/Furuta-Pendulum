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

void test_keepalive_freshness_boundaries_and_timer_wrap() {
  TEST_ASSERT_FALSE(furuta::keepaliveFresh(false, 1000U, 900U, 450U));
  TEST_ASSERT_TRUE(furuta::keepaliveFresh(true, 1350U, 900U, 450U));
  TEST_ASSERT_FALSE(furuta::keepaliveFresh(true, 1351U, 900U, 450U));
  TEST_ASSERT_TRUE(furuta::keepaliveFresh(true, 25U, UINT32_MAX - 24U, 50U));
  TEST_ASSERT_FALSE(furuta::keepaliveFresh(true, 26U, UINT32_MAX - 24U, 50U));
}

void test_projected_travel_counts_only_outward_motion() {
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6F, 1.4F, furuta::projectedAbsoluteTravel(1.0F, 2.0F, 0.2F));
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6F, 1.0F, furuta::projectedAbsoluteTravel(1.0F, -2.0F, 0.2F));
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6F, 1.4F, furuta::projectedAbsoluteTravel(-1.0F, -2.0F, 0.2F));
}

void test_down_angle_error_handles_wrap_boundary() {
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 0.0F,
                           furuta::downAngleError(-furuta::kPi));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 0.0F,
                           furuta::downAngleError(furuta::kPi));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, furuta::kPi,
                           furuta::downAngleError(0.0F));
}

void test_wrap_boundary_does_not_create_velocity_spike() {
  constexpr float epsilon = 0.001F;
  const float delta = furuta::wrapAngle(
      (-furuta::kPi + epsilon) - (furuta::kPi - epsilon));
  TEST_ASSERT_FLOAT_WITHIN(1.0e-6F, 2.0F * epsilon, delta);
}

void test_settling_filter_rejects_count_jitter_but_tracks_oscillation() {
  constexpr float dt_s = 0.005F;
  constexpr float settling_filter_hz = 3.0F;
  const float one_count_rate =
      (furuta::kTwoPi / 16384.0F) / dt_s;
  float filtered = 0.0F;
  float maximum_jitter = 0.0F;
  for (int sample = 0; sample < 400; ++sample) {
    const float raw = sample % 2 == 0 ? one_count_rate : -one_count_rate;
    filtered = furuta::lowPass(filtered, raw, settling_filter_hz, dt_s);
    maximum_jitter = std::fmax(maximum_jitter, std::fabs(filtered));
  }
  TEST_ASSERT_TRUE(maximum_jitter < 0.120F);

  filtered = 0.0F;
  float maximum_oscillation = 0.0F;
  for (int sample = 0; sample < 400; ++sample) {
    const float time_s = sample * dt_s;
    const float raw = 0.8F *
        std::cos(furuta::kTwoPi * 1.4F * time_s);
    filtered = furuta::lowPass(filtered, raw, settling_filter_hz, dt_s);
    if (sample > 200) {
      maximum_oscillation =
          std::fmax(maximum_oscillation, std::fabs(filtered));
    }
  }
  TEST_ASSERT_TRUE(maximum_oscillation > 0.5F);
}

void test_arm_settling_filter_rejects_single_velocity_spike() {
  constexpr float dt_s = 0.005F;
  constexpr float arm_settling_filter_hz = 5.0F;
  float filtered = furuta::lowPass(0.0F, 0.0822F,
                                   arm_settling_filter_hz, dt_s);
  TEST_ASSERT_TRUE(std::fabs(filtered) < 0.050F);
  for (int sample = 0; sample < 20; ++sample) {
    filtered = furuta::lowPass(filtered, 0.10F,
                               arm_settling_filter_hz, dt_s);
  }
  TEST_ASSERT_TRUE(std::fabs(filtered) > 0.050F);
}

void test_swing_preparation_requires_every_strict_settling_gate() {
  constexpr float arm_angle_tolerance = 0.020F;
  constexpr float arm_rate_tolerance = 0.050F;
  constexpr float down_angle_tolerance = 0.040F;
  constexpr float pendulum_rate_tolerance = 0.120F;
  TEST_ASSERT_TRUE(furuta::swingPreparationSettled(
      {0.0F, -furuta::kPi, 0.0F, 0.0F}, arm_angle_tolerance,
      arm_rate_tolerance, down_angle_tolerance,
      pendulum_rate_tolerance));
  TEST_ASSERT_TRUE(furuta::swingPreparationSettled(
      {0.0F, furuta::kPi, 0.0F, 0.0F}, arm_angle_tolerance,
      arm_rate_tolerance, down_angle_tolerance,
      pendulum_rate_tolerance));
  TEST_ASSERT_FALSE(furuta::swingPreparationSettled(
      {0.021F, -furuta::kPi, 0.0F, 0.0F}, arm_angle_tolerance,
      arm_rate_tolerance, down_angle_tolerance,
      pendulum_rate_tolerance));
  TEST_ASSERT_FALSE(furuta::swingPreparationSettled(
      {0.0F, -furuta::kPi, 0.051F, 0.0F}, arm_angle_tolerance,
      arm_rate_tolerance, down_angle_tolerance,
      pendulum_rate_tolerance));
  TEST_ASSERT_FALSE(furuta::swingPreparationSettled(
      {0.0F, -furuta::kPi + 0.041F, 0.0F, 0.0F}, arm_angle_tolerance,
      arm_rate_tolerance, down_angle_tolerance,
      pendulum_rate_tolerance));
  TEST_ASSERT_FALSE(furuta::swingPreparationSettled(
      {0.0F, -furuta::kPi, 0.0F, 0.121F}, arm_angle_tolerance,
      arm_rate_tolerance, down_angle_tolerance,
      pendulum_rate_tolerance));
}

void test_gain_limits_follow_the_reviewed_model_profile() {
  const furuta::Gains limits{0.16F, 2.5F, 0.13F, 0.25F};
  const furuta::Gains model{-0.0914F, 1.4443F, -0.0692F, 0.1389F};
  TEST_ASSERT_TRUE(furuta::gainsWithinAbsoluteLimits(model, limits));
  TEST_ASSERT_TRUE(furuta::gainsWithinScaledProfile(
      {-0.05941F, 0.93881F, -0.04498F, 0.09026F}, model, 0.5F, 1.5F));
  TEST_ASSERT_FALSE(furuta::gainsWithinScaledProfile(
      {0.0914F, -1.4443F, 0.0692F, -0.1389F}, model, 0.5F, 1.5F));
  TEST_ASSERT_FALSE(furuta::gainsWithinScaledProfile(
      {-0.04F, 1.4443F, -0.0692F, 0.1389F}, model, 0.5F, 1.5F));
  TEST_ASSERT_FALSE(furuta::gainsWithinAbsoluteLimits(
      {-0.2F, 1.4443F, -0.0692F, 0.1389F}, limits));
}

void test_swing_settings_reject_negative_non_finite_and_excessive_values() {
  const furuta::SwingSettings limits{20.0F, 1.0F, 1.0F, 0.30F, 0.45F};
  TEST_ASSERT_TRUE(furuta::swingSettingsWithinLimits(
      {0.8F, 0.03F, 0.04F, 0.18F, 0.45F}, limits));
  TEST_ASSERT_FALSE(furuta::swingSettingsWithinLimits(
      {-0.1F, 0.03F, 0.04F, 0.18F, 0.45F}, limits));
  TEST_ASSERT_FALSE(furuta::swingSettingsWithinLimits(
      {20.1F, 0.03F, 0.04F, 0.18F, 0.45F}, limits));
  TEST_ASSERT_FALSE(furuta::swingSettingsWithinLimits(
      {NAN, 0.03F, 0.04F, 0.18F, 0.45F}, limits));
  TEST_ASSERT_FALSE(furuta::swingSettingsWithinLimits(
      {0.8F, 0.03F, 0.04F, 0.18F, 0.451F}, limits));
}

void test_swing_energy_law_pumps_in_the_direction_of_motion() {
  const float positive_torque = furuta::swingUpTorque(
      {0.0F, -furuta::kPi + 0.1F, 0.0F, -1.0F},
      0.159F, 0.05379F, 0.001141F, 0.8F, 0.0F);
  const float negative_torque = furuta::swingUpTorque(
      {0.0F, -furuta::kPi + 0.1F, 0.0F, 1.0F},
      0.159F, 0.05379F, 0.001141F, 0.8F, 0.0F);
  TEST_ASSERT_TRUE(positive_torque > 0.0F);
  TEST_ASSERT_TRUE(negative_torque < 0.0F);
  TEST_ASSERT_FLOAT_WITHIN(
      1.0e-6F, 0.0F,
      furuta::swingUpTorque({0.0F, 0.0F, 0.0F, 0.0F},
                            0.159F, 0.05379F, 0.001141F, 0.8F, 0.0F));
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

void test_as5048a_diagnostics_register_and_flags() {
  TEST_ASSERT_EQUAL_HEX16(
      as5048a::withEvenParity(as5048a::kReadFlag | 0x3FFDU),
      as5048a::makeReadCommand(as5048a::kRegisterDiagnostics));
  const auto weak = as5048a::decodeDiagnostics(
      static_cast<uint16_t>((1U << 8) | (1U << 10) | 200U));
  TEST_ASSERT_EQUAL_UINT8(200U, weak.agc);
  TEST_ASSERT_TRUE(weak.offset_compensation_finished);
  TEST_ASSERT_TRUE(weak.magnetic_field_too_weak);
  TEST_ASSERT_FALSE(weak.magnetic_field_too_strong);
  const auto strong = as5048a::decodeDiagnostics(
      static_cast<uint16_t>((1U << 8) | (1U << 11) | 12U));
  TEST_ASSERT_FALSE(strong.magnetic_field_too_weak);
  TEST_ASSERT_TRUE(strong.magnetic_field_too_strong);
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

void test_feedback_parser_rejects_non_finite_values() {
  char nan_payload[] = "nan 0.0";
  char inf_payload[] = "0.0 inf";
  float position = 0.0F;
  float velocity = 0.0F;
  TEST_ASSERT_FALSE(
      odrive_ascii::parseFeedback(nan_payload, position, velocity));
  TEST_ASSERT_FALSE(
      odrive_ascii::parseFeedback(inf_payload, position, velocity));
}

void test_inactive_feedback_poll_never_runs_during_active_control() {
  TEST_ASSERT_FALSE(odrive_ascii::inactiveFeedbackDue(true, 100U, 0U, 20U));
  TEST_ASSERT_FALSE(odrive_ascii::inactiveFeedbackDue(false, 19U, 0U, 20U));
  TEST_ASSERT_TRUE(odrive_ascii::inactiveFeedbackDue(false, 20U, 0U, 20U));
  TEST_ASSERT_TRUE(odrive_ascii::inactiveFeedbackDue(
      false, 10U, UINT32_MAX - 14U, 20U));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_wrap_angle_range_and_boundaries);
  RUN_TEST(test_balance_torque_uses_documented_state_order);
  RUN_TEST(test_slew_limit_bounds_both_directions);
  RUN_TEST(test_keepalive_freshness_boundaries_and_timer_wrap);
  RUN_TEST(test_projected_travel_counts_only_outward_motion);
  RUN_TEST(test_down_angle_error_handles_wrap_boundary);
  RUN_TEST(test_wrap_boundary_does_not_create_velocity_spike);
  RUN_TEST(test_settling_filter_rejects_count_jitter_but_tracks_oscillation);
  RUN_TEST(test_arm_settling_filter_rejects_single_velocity_spike);
  RUN_TEST(test_swing_preparation_requires_every_strict_settling_gate);
  RUN_TEST(test_gain_limits_follow_the_reviewed_model_profile);
  RUN_TEST(test_swing_settings_reject_negative_non_finite_and_excessive_values);
  RUN_TEST(test_swing_energy_law_pumps_in_the_direction_of_motion);
  RUN_TEST(test_as5048a_command_and_response_parity);
  RUN_TEST(test_as5048a_diagnostics_register_and_flags);
  RUN_TEST(test_ascii_checksum_matches_odrive_documentation);
  RUN_TEST(test_ascii_response_validation_and_feedback_parse);
  RUN_TEST(test_ascii_checksum_rejects_corruption);
  RUN_TEST(test_feedback_parser_rejects_non_finite_values);
  RUN_TEST(test_inactive_feedback_poll_never_runs_during_active_control);
  return UNITY_END();
}
