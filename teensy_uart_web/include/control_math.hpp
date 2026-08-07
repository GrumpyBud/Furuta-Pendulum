#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

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

inline bool angularStepPlausible(const float previous_angle_rad,
                                 const float candidate_angle_rad,
                                 const float elapsed_s,
                                 const float maximum_velocity_rad_s) {
  if (!std::isfinite(previous_angle_rad) ||
      !std::isfinite(candidate_angle_rad) || !std::isfinite(elapsed_s) ||
      !std::isfinite(maximum_velocity_rad_s) || elapsed_s <= 0.0F ||
      maximum_velocity_rad_s <= 0.0F) {
    return false;
  }
  return std::fabs(wrapAngle(candidate_angle_rad - previous_angle_rad)) <=
         maximum_velocity_rad_s * elapsed_s;
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

inline float torqueModeVelocityAllowance(const float velocity_turns_s,
                                         const float velocity_limit_turns_s,
                                         const float velocity_gain) {
  return std::fmax(
      0.0F,
      velocity_gain * (velocity_limit_turns_s -
                       std::fabs(velocity_turns_s)));
}

inline bool keepaliveFresh(const bool held, const uint32_t now,
                           const uint32_t last, const uint32_t timeout) {
  return held && now - last <= timeout;
}

inline float projectedAbsoluteTravel(const float angle_rad,
                                     const float velocity_rad_s,
                                     const float horizon_s) {
  const float outward_velocity =
      angle_rad > 0.0F
          ? std::fmax(velocity_rad_s, 0.0F)
          : (angle_rad < 0.0F ? std::fmax(-velocity_rad_s, 0.0F)
                              : std::fabs(velocity_rad_s));
  return std::fabs(angle_rad) + outward_velocity * horizon_s;
}

inline float downAngleError(const float angle_rad) {
  return std::fabs(std::fabs(wrapAngle(angle_rad)) - kPi);
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

struct SwingSettings {
  float energy_gain;
  float arm_damping;
  float arm_centering;
  float startup_kick_nm;
  float torque_limit_nm;
};

inline bool swingSettingsWithinLimits(const SwingSettings& candidate,
                                      const SwingSettings& limits) {
  const float candidate_values[] = {
      candidate.energy_gain, candidate.arm_damping,
      candidate.arm_centering, candidate.startup_kick_nm,
      candidate.torque_limit_nm};
  const float limit_values[] = {
      limits.energy_gain, limits.arm_damping,
      limits.arm_centering, limits.startup_kick_nm,
      limits.torque_limit_nm};
  for (size_t index = 0; index < 5U; ++index) {
    if (!std::isfinite(candidate_values[index]) ||
        candidate_values[index] < 0.0F ||
        candidate_values[index] > limit_values[index]) {
      return false;
    }
  }
  return true;
}

inline bool gainsWithinAbsoluteLimits(const Gains& candidate,
                                      const Gains& limits) {
  return std::isfinite(candidate.arm_angle) &&
         std::isfinite(candidate.pendulum_angle) &&
         std::isfinite(candidate.arm_velocity) &&
         std::isfinite(candidate.pendulum_velocity) &&
         std::fabs(candidate.arm_angle) <= std::fabs(limits.arm_angle) &&
         std::fabs(candidate.pendulum_angle) <=
             std::fabs(limits.pendulum_angle) &&
         std::fabs(candidate.arm_velocity) <=
             std::fabs(limits.arm_velocity) &&
         std::fabs(candidate.pendulum_velocity) <=
             std::fabs(limits.pendulum_velocity);
}

inline bool gainsWithinScaledProfile(const Gains& candidate,
                                     const Gains& reference,
                                     const float minimum_scale,
                                     const float maximum_scale) {
  if (!(minimum_scale > 0.0F && maximum_scale >= minimum_scale)) return false;
  const float reference_values[] = {
      reference.arm_angle, reference.pendulum_angle,
      reference.arm_velocity, reference.pendulum_velocity};
  const float candidate_values[] = {
      candidate.arm_angle, candidate.pendulum_angle,
      candidate.arm_velocity, candidate.pendulum_velocity};
  for (size_t index = 0; index < 4U; ++index) {
    if (!std::isfinite(reference_values[index]) ||
        !std::isfinite(candidate_values[index]) ||
        reference_values[index] == 0.0F) {
      return false;
    }
    const float scale = candidate_values[index] / reference_values[index];
    if (scale < minimum_scale || scale > maximum_scale) return false;
  }
  return true;
}

inline float balanceTorque(const State& state, const Gains& gains) {
  return -(gains.arm_angle * state.arm_angle_rad +
           gains.pendulum_angle * state.pendulum_angle_rad +
           gains.arm_velocity * state.arm_velocity_rad_s +
           gains.pendulum_velocity * state.pendulum_velocity_rad_s);
}

inline bool swingPreparationSettled(
    const State& state, const float arm_angle_tolerance_rad,
    const float arm_rate_tolerance_rad_s,
    const float down_angle_tolerance_rad,
    const float pendulum_rate_tolerance_rad_s) {
  return std::fabs(state.arm_angle_rad) < arm_angle_tolerance_rad &&
         std::fabs(state.arm_velocity_rad_s) < arm_rate_tolerance_rad_s &&
         downAngleError(state.pendulum_angle_rad) <
             down_angle_tolerance_rad &&
         std::fabs(state.pendulum_velocity_rad_s) <
             pendulum_rate_tolerance_rad_s;
}

inline float swingUpTorque(const State& state, const float mass_kg,
                           const float com_length_m,
                           const float inertia_kg_m2,
                           const float energy_gain,
                           const float arm_damping,
                           const float target_energy_j = 0.0F) {
  constexpr float gravity_m_s2 = 9.80665F;
  const float energy =
      0.5F * inertia_kg_m2 * state.pendulum_velocity_rad_s *
          state.pendulum_velocity_rad_s +
      mass_kg * gravity_m_s2 * com_length_m *
          (std::cos(state.pendulum_angle_rad) - 1.0F);
  return energy_gain * (target_energy_j - energy) *
             state.pendulum_velocity_rad_s *
             std::cos(state.pendulum_angle_rad) -
         arm_damping * state.arm_velocity_rad_s;
}

inline float swingApproachEnergyTarget(
    const State& state, const float reserve_j_per_rad_s,
    const float maximum_reserve_j) {
  const bool approaching_upright =
      std::fabs(state.pendulum_angle_rad) < 0.5F * kPi &&
      state.pendulum_angle_rad * state.pendulum_velocity_rad_s < 0.0F;
  if (!approaching_upright) return 0.0F;
  return -std::fmin(maximum_reserve_j,
                    reserve_j_per_rad_s *
                        std::fabs(state.pendulum_velocity_rad_s));
}

inline float outwardSwingScale(const float arm_angle_rad,
                               const float energy_torque_nm,
                               const float guard_start_rad,
                               const float guard_full_rad) {
  if (arm_angle_rad * energy_torque_nm <= 0.0F) return 1.0F;
  if (!(guard_full_rad > guard_start_rad)) return 0.0F;
  return clamp((guard_full_rad - std::fabs(arm_angle_rad)) /
                   (guard_full_rad - guard_start_rad),
               0.0F, 1.0F);
}

inline float travelAwareSwingUpTorque(
    const State& state, const float mass_kg, const float com_length_m,
    const float inertia_kg_m2, const float energy_gain,
    const float arm_damping, const float arm_centering,
    const float guard_start_rad, const float guard_full_rad,
    const float approach_reserve_j_per_rad_s = 0.0F,
    const float maximum_approach_reserve_j = 0.0F) {
  // Preserve full energy pumping near center and whenever it helps return the
  // arm. Only fade the component that would push an already displaced arm
  // farther outward; the restoring PD terms always retain authority.
  const float target_energy_j = swingApproachEnergyTarget(
      state, approach_reserve_j_per_rad_s,
      maximum_approach_reserve_j);
  const float energy_torque =
      swingUpTorque(state, mass_kg, com_length_m, inertia_kg_m2,
                    energy_gain, 0.0F, target_energy_j);
  const float guarded_energy_torque =
      energy_torque * outwardSwingScale(
                          state.arm_angle_rad, energy_torque,
                          guard_start_rad, guard_full_rad);
  return guarded_energy_torque - arm_damping * state.arm_velocity_rad_s -
         arm_centering * state.arm_angle_rad;
}

}  // namespace furuta
