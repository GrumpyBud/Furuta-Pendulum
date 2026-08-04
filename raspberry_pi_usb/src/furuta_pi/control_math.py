"""Pure control functions shared by the runtime and unit tests."""

from __future__ import annotations

from dataclasses import dataclass
import math

PI = math.pi
TWO_PI = 2.0 * PI


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def wrap_angle(angle: float) -> float:
    """Return an equivalent angle in the half-open interval [-pi, pi)."""
    return (angle + PI) % TWO_PI - PI


def low_pass(previous: float, sample: float, cutoff_hz: float, dt_s: float) -> float:
    alpha = 1.0 - math.exp(-TWO_PI * cutoff_hz * dt_s)
    return previous + alpha * (sample - previous)


@dataclass(frozen=True, slots=True)
class State:
    arm_angle_rad: float
    pendulum_angle_rad: float
    arm_velocity_rad_s: float
    pendulum_velocity_rad_s: float


@dataclass(frozen=True, slots=True)
class Gains:
    arm_angle: float
    pendulum_angle: float
    arm_velocity: float
    pendulum_velocity: float


def balance_torque(state: State, gains: Gains) -> float:
    return -(
        gains.arm_angle * state.arm_angle_rad
        + gains.pendulum_angle * state.pendulum_angle_rad
        + gains.arm_velocity * state.arm_velocity_rad_s
        + gains.pendulum_velocity * state.pendulum_velocity_rad_s
    )


def swing_up_torque(
    state: State,
    mass_kg: float,
    com_length_m: float,
    inertia_kg_m2: float,
    energy_gain: float,
    arm_damping: float,
) -> float:
    gravity_m_s2 = 9.80665
    energy = (
        0.5 * inertia_kg_m2 * state.pendulum_velocity_rad_s**2
        + mass_kg
        * gravity_m_s2
        * com_length_m
        * (math.cos(state.pendulum_angle_rad) - 1.0)
    )
    return (
        energy_gain
        * (-energy)
        * state.pendulum_velocity_rad_s
        * math.cos(state.pendulum_angle_rad)
        - arm_damping * state.arm_velocity_rad_s
    )
