#include <unity.h>

#include "control_math.hpp"

void test_wrap_angle_boundaries() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, furuta::wrapAngle(0.0F));
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, -furuta::kPi,
                           furuta::wrapAngle(furuta::kPi));
  TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.25F,
                           furuta::wrapAngle(2.0F * furuta::kPi + 0.25F));
}

void test_clamp() {
  TEST_ASSERT_EQUAL_FLOAT(-1.0F, furuta::clamp(-2.0F, -1.0F, 1.0F));
  TEST_ASSERT_EQUAL_FLOAT(0.2F, furuta::clamp(0.2F, -1.0F, 1.0F));
  TEST_ASSERT_EQUAL_FLOAT(1.0F, furuta::clamp(2.0F, -1.0F, 1.0F));
}

void test_balance_feedback_sign_and_sum() {
  const furuta::State x{1.0F, 2.0F, 3.0F, 4.0F};
  const furuta::Gains k{1.0F, 10.0F, 100.0F, 1000.0F};
  TEST_ASSERT_EQUAL_FLOAT(-4321.0F, furuta::balanceTorque(x, k));
}

void test_swing_up_is_zero_at_target_energy() {
  const furuta::State upright_at_rest{0.0F, 0.0F, 0.0F, 0.0F};
  TEST_ASSERT_FLOAT_WITHIN(
      1e-7F, 0.0F,
      furuta::swingUpTorque(upright_at_rest, 0.1F, 0.2F, 0.003F, 5.0F,
                            0.1F));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_wrap_angle_boundaries);
  RUN_TEST(test_clamp);
  RUN_TEST(test_balance_feedback_sign_and_sum);
  RUN_TEST(test_swing_up_is_zero_at_target_energy);
  return UNITY_END();
}

