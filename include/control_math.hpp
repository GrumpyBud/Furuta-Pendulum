#pragma once

#include <cmath>

namespace furuta {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

inline float clamp(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}

// Returns an equivalent angle in [-pi, pi). Keeping this convention in one
// function prevents a surprising number of sign and discontinuity mistakes.
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

struct State {
  float arm_angle_rad;
  float pendulum_angle_rad;  // zero upright, positive in encoder direction
  float arm_velocity_rad_s;
  float pendulum_velocity_rad_s;
};

struct Gains {
  float arm_angle;
  float pendulum_angle;
  float arm_velocity;
  float pendulum_velocity;
};

inline float balanceTorque(const State& x, const Gains& k) {
  return -(k.arm_angle * x.arm_angle_rad +
           k.pendulum_angle * x.pendulum_angle_rad +
           k.arm_velocity * x.arm_velocity_rad_s +
           k.pendulum_velocity * x.pendulum_velocity_rad_s);
}

// Energy shaping for a pendulum whose angle is zero at upright. The constant
// m*g*l is the potential-energy scale, with l measured to the centre of mass.
// Multiplying by alpha_dot*cos(alpha) chooses arm acceleration that pumps
// energy in regardless of which way the pendulum is moving.
inline float swingUpTorque(const State& x, const float mass_kg,
                           const float com_length_m,
                           const float inertia_kg_m2,
                           const float energy_gain,
                           const float arm_damping) {
  constexpr float gravity_m_s2 = 9.80665F;
  const float energy = 0.5F * inertia_kg_m2 *
                           x.pendulum_velocity_rad_s *
                           x.pendulum_velocity_rad_s +
                       mass_kg * gravity_m_s2 * com_length_m *
                           (std::cos(x.pendulum_angle_rad) - 1.0F);
  return energy_gain * (-energy) * x.pendulum_velocity_rad_s *
             std::cos(x.pendulum_angle_rad) -
         arm_damping * x.arm_velocity_rad_s;
}

}  // namespace furuta

