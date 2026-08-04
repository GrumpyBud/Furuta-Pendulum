#pragma once

#include <cmath>

namespace furuta {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

inline float clamp(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}

inline float wrapAngle(float angle) {
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0F) angle += kTwoPi;
  return angle - kPi;
}

inline float lowPass(const float previous, const float sample,
                     const float cutoff_hz, const float dt_s) {
  const float alpha = 1.0F - std::exp(-kTwoPi * cutoff_hz * dt_s);
  return previous + alpha * (sample - previous);
}

inline float slewLimit(const float previous, const float requested,
                       const float maximum_change) {
  return clamp(requested, previous - maximum_change,
               previous + maximum_change);
}

struct State {
  float arm_angle_rad;
  float pendulum_angle_rad;  // zero upright; hanging down is approximately -pi
  float arm_velocity_rad_s;
  float pendulum_velocity_rad_s;
};

struct Gains {
  float arm_angle;
  float pendulum_angle;
  float arm_velocity;
  float pendulum_velocity;
};

inline float balanceTorque(const State& state, const Gains& gains) {
  return -(gains.arm_angle * state.arm_angle_rad +
           gains.pendulum_angle * state.pendulum_angle_rad +
           gains.arm_velocity * state.arm_velocity_rad_s +
           gains.pendulum_velocity * state.pendulum_velocity_rad_s);
}

inline float swingUpTorque(const State& state, const float mass_kg,
                           const float com_length_m,
                           const float inertia_kg_m2,
                           const float energy_gain,
                           const float arm_damping) {
  constexpr float gravity_m_s2 = 9.80665F;
  const float energy =
      0.5F * inertia_kg_m2 * state.pendulum_velocity_rad_s *
          state.pendulum_velocity_rad_s +
      mass_kg * gravity_m_s2 * com_length_m *
          (std::cos(state.pendulum_angle_rad) - 1.0F);
  return energy_gain * (-energy) * state.pendulum_velocity_rad_s *
             std::cos(state.pendulum_angle_rad) -
         arm_damping * state.arm_velocity_rad_s;
}

}  // namespace furuta
