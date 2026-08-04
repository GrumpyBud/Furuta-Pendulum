"""Interactive best-effort control loop for Raspberry Pi OS."""

from __future__ import annotations

import asyncio
from enum import Enum, auto
import math
import sys
from typing import Protocol

from .config import AppConfig
from .control_math import (
    PI,
    TWO_PI,
    State,
    balance_torque,
    clamp,
    low_pass,
    swing_up_torque,
    wrap_angle,
)
from .odrive_usb import DriveFeedback, ODriveUsb


class Inputs(Protocol):
    @property
    def pendulum_steps(self) -> int: ...

    @property
    def estop_closed(self) -> bool: ...


class Mode(Enum):
    DISARMED = auto()
    SWING_UP = auto()
    BALANCE = auto()
    FAULT = auto()


class FurutaController:
    def __init__(
        self, config: AppConfig, inputs: Inputs, drive: ODriveUsb
    ) -> None:
        self.config = config
        self.inputs = inputs
        self.drive = drive
        self.mode = Mode.DISARMED
        self._zero_valid = False
        self._pendulum_down_steps = 0
        self._arm_zero_turns = 0.0
        self._last_pendulum_angle = -PI
        self._pendulum_velocity = 0.0
        self._last_state: State | None = None
        self._last_feedback: DriveFeedback | None = None
        self._command_queue: asyncio.Queue[str] = asyncio.Queue()
        self._stop_requested = False
        self._fault_reason = ""
        self._last_loop_dt_ms = 0.0
        self._max_lateness_ms = 0.0

    def submit_command(self, command: str) -> None:
        self._command_queue.put_nowait(command.strip().lower())

    def request_stop(self) -> None:
        self._stop_requested = True

    def _sample_state(self, feedback: DriveFeedback, dt_s: float) -> State:
        hardware = self.config.hardware
        relative_steps = self.inputs.pendulum_steps - self._pendulum_down_steps
        step_to_rad = (
            hardware.pendulum_direction
            * TWO_PI
            / hardware.pendulum_steps_per_revolution
        )
        pendulum_angle = wrap_angle(relative_steps * step_to_rad - PI)
        raw_velocity = wrap_angle(
            pendulum_angle - self._last_pendulum_angle
        ) / max(dt_s, 1.0e-6)
        self._pendulum_velocity = low_pass(
            self._pendulum_velocity,
            raw_velocity,
            self.config.control.velocity_filter_hz,
            dt_s,
        )
        self._last_pendulum_angle = pendulum_angle
        arm_scale = (
            hardware.motor_turns_to_arm_radians * hardware.motor_direction
        )
        return State(
            (feedback.position_turns - self._arm_zero_turns) * arm_scale,
            pendulum_angle,
            feedback.velocity_turns_s * arm_scale,
            self._pendulum_velocity,
        )

    def _health_problem(self, state: State, feedback: DriveFeedback) -> str | None:
        limits = self.config.control
        if not self.inputs.estop_closed:
            return "E-stop loop open"
        if feedback.active_errors != 0:
            return f"ODrive active error 0x{feedback.active_errors:X}"
        if feedback.current_state != self.drive.closed_loop_state_value():
            return (
                f"ODrive left closed loop (state {feedback.current_state}, "
                f"disarm reason 0x{feedback.disarm_reason:X})"
            )
        if not all(
            math.isfinite(value)
            for value in (
                state.arm_angle_rad,
                state.pendulum_angle_rad,
                state.arm_velocity_rad_s,
                state.pendulum_velocity_rad_s,
            )
        ):
            return "non-finite sensor value"
        if abs(state.arm_angle_rad) > limits.arm_angle_limit_rad:
            return "arm travel limit"
        if abs(state.arm_velocity_rad_s) > limits.arm_velocity_limit_rad_s:
            return "arm overspeed"
        if abs(state.pendulum_velocity_rad_s) > limits.pendulum_velocity_limit_rad_s:
            return "pendulum overspeed"
        return None

    def _calculate_torque(self, state: State) -> float:
        control = self.config.control
        if (
            self.mode is Mode.SWING_UP
            and abs(state.pendulum_angle_rad) < control.catch_angle_rad
            and abs(state.pendulum_velocity_rad_s)
            < control.catch_pendulum_velocity_rad_s
        ):
            self.mode = Mode.BALANCE
        elif (
            self.mode is Mode.BALANCE
            and abs(state.pendulum_angle_rad) > control.drop_angle_rad
        ):
            self.mode = Mode.SWING_UP

        if self.mode is Mode.BALANCE:
            torque = balance_torque(state, control.balance_gains)
        else:
            torque = swing_up_torque(
                state,
                control.pendulum_mass_kg,
                control.pendulum_com_length_m,
                control.pendulum_inertia_kg_m2,
                control.swing_energy_gain,
                control.swing_arm_damping,
            )
            torque = clamp(
                torque,
                -control.swing_torque_limit_nm,
                control.swing_torque_limit_nm,
            )
        return clamp(torque, -control.torque_limit_nm, control.torque_limit_nm)

    async def _fault(self, reason: str) -> None:
        if self.mode is Mode.FAULT:
            return
        self.mode = Mode.FAULT
        self._fault_reason = reason
        print(f"\nFAULT: {reason}", flush=True)
        await self.drive.safe_idle()

    async def _execute_command(self, command: str) -> None:
        if not command:
            return
        if command == "zero" and self.mode is Mode.DISARMED:
            if self._last_feedback is None:
                print("Refused: no ODrive feedback yet.")
                return
            self._pendulum_down_steps = self.inputs.pendulum_steps
            self._arm_zero_turns = self._last_feedback.position_turns
            self._last_pendulum_angle = -PI
            self._pendulum_velocity = 0.0
            self._zero_valid = True
            print("Zero saved: pendulum must be hanging straight down.")
        elif command == "arm" and self.mode is Mode.DISARMED:
            if not self._zero_valid:
                print("Refused: send zero with the pendulum hanging down first.")
            elif not self.inputs.estop_closed:
                print("Refused: close the safety loop first.")
            elif self._last_feedback is None:
                print("Refused: no ODrive feedback yet.")
            elif self._last_feedback.active_errors != 0:
                print(
                    "Refused: ODrive reports active errors "
                    f"0x{self._last_feedback.active_errors:X}."
                )
            else:
                print("Arming torque control...", flush=True)
                await self.drive.arm_torque_control()
                feedback = await self.drive.read_feedback()
                if feedback.current_state != self.drive.closed_loop_state_value():
                    await self._fault("ODrive refused closed-loop control")
                else:
                    self._last_feedback = feedback
                    self.mode = Mode.SWING_UP
                    self._max_lateness_ms = 0.0
                    print("ARMED. Keep clear; type disarm to stop.")
        elif command == "disarm":
            await self.drive.safe_idle()
            self.mode = Mode.DISARMED
            self._fault_reason = ""
            print("Disarmed.")
        elif command == "status":
            self.print_status()
        elif command == "help":
            print("Commands: zero, arm, disarm, status, help, quit")
        elif command in {"quit", "exit"}:
            self._stop_requested = True
        else:
            print("Unknown or invalid command. Type help.")

    def print_status(self) -> None:
        if self._last_state is None:
            print(f"mode={self.mode.name} waiting_for_feedback")
            return
        state = self._last_state
        suffix = f" fault={self._fault_reason}" if self._fault_reason else ""
        print(
            f"mode={self.mode.name} "
            f"arm={state.arm_angle_rad:.4f} "
            f"pend={state.pendulum_angle_rad:.4f} "
            f"arm_rate={state.arm_velocity_rad_s:.3f} "
            f"pend_rate={state.pendulum_velocity_rad_s:.3f} "
            f"loop_dt_ms={self._last_loop_dt_ms:.2f} "
            f"max_late_ms={self._max_lateness_ms:.2f}"
            f"{suffix}",
            flush=True,
        )

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        period_s = 1.0 / self.config.control.control_hz
        next_deadline = loop.time()
        last_sample_time = next_deadline
        last_telemetry_time = next_deadline
        print("Furuta Pi controller ready. DISARMED. Type help.", flush=True)

        while not self._stop_requested:
            processed_command = False
            while not self._command_queue.empty():
                await self._execute_command(self._command_queue.get_nowait())
                processed_command = True
            if processed_command:
                # Arming and status commands involve USB I/O and console output;
                # do not misclassify their latency as a missed control deadline.
                next_deadline = loop.time()
                last_sample_time = next_deadline

            try:
                feedback = await self.drive.read_feedback()
            except Exception as error:
                if self.mode not in {Mode.DISARMED, Mode.FAULT}:
                    await self._fault(f"USB feedback failed: {error}")
                raise RuntimeError(f"ODrive USB connection failed: {error}") from error

            now = loop.time()
            dt_s = max(now - last_sample_time, 1.0e-6)
            last_sample_time = now
            self._last_loop_dt_ms = dt_s * 1000.0
            self._last_feedback = feedback
            self._last_state = self._sample_state(feedback, dt_s)

            if self.mode in {Mode.SWING_UP, Mode.BALANCE}:
                problem = self._health_problem(self._last_state, feedback)
                if problem:
                    await self._fault(problem)
                else:
                    torque = self._calculate_torque(self._last_state)
                    try:
                        await self.drive.write_torque(torque)
                    except Exception as error:
                        await self._fault(f"USB torque write failed: {error}")

            if now - last_telemetry_time >= self.config.control.telemetry_interval_s:
                last_telemetry_time = now
                self.print_status()

            next_deadline += period_s
            lateness = loop.time() - next_deadline
            self._max_lateness_ms = max(self._max_lateness_ms, lateness * 1000.0)
            if (
                lateness > self.config.control.max_control_lateness_s
                and self.mode in {Mode.SWING_UP, Mode.BALANCE}
            ):
                await self._fault(f"control loop late by {lateness * 1000.0:.1f} ms")
            if lateness > period_s:
                next_deadline = loop.time()
            else:
                await asyncio.sleep(max(0.0, next_deadline - loop.time()))

        await self.drive.safe_idle()


def install_console(controller: FurutaController) -> None:
    """Connect newline commands from stdin to the asyncio controller."""
    loop = asyncio.get_running_loop()

    def read_command() -> None:
        line = sys.stdin.readline()
        if line:
            controller.submit_command(line)
        else:
            controller.request_stop()

    loop.add_reader(sys.stdin.fileno(), read_command)
