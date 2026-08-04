"""TOML configuration loading and validation."""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
import tomllib
from typing import Any, TypeVar

from .control_math import Gains


@dataclass(frozen=True, slots=True)
class HardwareConfig:
    encoder_a_bcm: int = 17
    encoder_b_bcm: int = 27
    estop_bcm: int = 22
    gpio_chip: int = 0
    pendulum_steps_per_revolution: int = 2048
    pendulum_direction: float = 1.0
    motor_turns_to_arm_radians: float = 6.283185307179586
    motor_direction: float = 1.0


@dataclass(frozen=True, slots=True)
class ODriveConfig:
    serial_number: str = ""
    watchdog_timeout_s: float = 0.10
    usb_operation_timeout_s: float = 0.030


@dataclass(frozen=True, slots=True)
class ControlConfig:
    control_hz: float = 200.0
    max_control_lateness_s: float = 0.020
    telemetry_interval_s: float = 1.0
    velocity_filter_hz: float = 45.0
    torque_limit_nm: float = 1.5
    arm_angle_limit_rad: float = 2.6
    arm_velocity_limit_rad_s: float = 25.0
    pendulum_velocity_limit_rad_s: float = 60.0
    pendulum_mass_kg: float = 0.125
    pendulum_com_length_m: float = 0.145
    pendulum_inertia_kg_m2: float = 0.0035
    swing_energy_gain: float = 7.0
    swing_arm_damping: float = 0.08
    swing_torque_limit_nm: float = 0.65
    catch_angle_rad: float = 0.22
    drop_angle_rad: float = 0.45
    catch_pendulum_velocity_rad_s: float = 3.0
    balance_arm_angle_gain: float = 1.8
    balance_pendulum_angle_gain: float = -18.0
    balance_arm_velocity_gain: float = 1.25
    balance_pendulum_velocity_gain: float = -2.2

    @property
    def balance_gains(self) -> Gains:
        return Gains(
            self.balance_arm_angle_gain,
            self.balance_pendulum_angle_gain,
            self.balance_arm_velocity_gain,
            self.balance_pendulum_velocity_gain,
        )


@dataclass(frozen=True, slots=True)
class AppConfig:
    hardware: HardwareConfig
    odrive: ODriveConfig
    control: ControlConfig


ConfigType = TypeVar("ConfigType", HardwareConfig, ODriveConfig, ControlConfig)


def _section(cls: type[ConfigType], raw: dict[str, Any], name: str) -> ConfigType:
    unknown = set(raw) - {field.name for field in fields(cls)}
    if unknown:
        raise ValueError(f"unknown {name} setting(s): {', '.join(sorted(unknown))}")
    return cls(**raw)


def load_config(path: str | Path) -> AppConfig:
    with Path(path).open("rb") as config_file:
        raw = tomllib.load(config_file)
    unknown_sections = set(raw) - {"hardware", "odrive", "control"}
    if unknown_sections:
        raise ValueError(
            f"unknown config section(s): {', '.join(sorted(unknown_sections))}"
        )
    config = AppConfig(
        hardware=_section(HardwareConfig, raw.get("hardware", {}), "hardware"),
        odrive=_section(ODriveConfig, raw.get("odrive", {}), "odrive"),
        control=_section(ControlConfig, raw.get("control", {}), "control"),
    )
    validate_config(config)
    return config


def validate_config(config: AppConfig) -> None:
    hardware = config.hardware
    control = config.control
    odrive = config.odrive
    pins = (hardware.encoder_a_bcm, hardware.encoder_b_bcm, hardware.estop_bcm)
    if len(set(pins)) != len(pins) or any(pin < 0 for pin in pins):
        raise ValueError("encoder and E-stop BCM GPIO numbers must be distinct and nonnegative")
    if hardware.pendulum_steps_per_revolution <= 0:
        raise ValueError("pendulum_steps_per_revolution must be positive")
    if hardware.pendulum_direction not in (-1.0, 1.0):
        raise ValueError("pendulum_direction must be -1.0 or 1.0")
    if hardware.motor_direction not in (-1.0, 1.0):
        raise ValueError("motor_direction must be -1.0 or 1.0")
    positive_values = {
        "control_hz": control.control_hz,
        "max_control_lateness_s": control.max_control_lateness_s,
        "telemetry_interval_s": control.telemetry_interval_s,
        "velocity_filter_hz": control.velocity_filter_hz,
        "torque_limit_nm": control.torque_limit_nm,
        "watchdog_timeout_s": odrive.watchdog_timeout_s,
        "usb_operation_timeout_s": odrive.usb_operation_timeout_s,
    }
    invalid = [name for name, value in positive_values.items() if value <= 0.0]
    if invalid:
        raise ValueError(f"setting(s) must be positive: {', '.join(invalid)}")
    if odrive.watchdog_timeout_s <= odrive.usb_operation_timeout_s:
        raise ValueError("watchdog_timeout_s must exceed usb_operation_timeout_s")
    if control.catch_angle_rad >= control.drop_angle_rad:
        raise ValueError("catch_angle_rad must be smaller than drop_angle_rad")
