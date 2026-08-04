"""Small async adapter around ODrive's native USB/Fibre interface."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any

from .config import ODriveConfig


@dataclass(frozen=True, slots=True)
class DriveFeedback:
    position_turns: float
    velocity_turns_s: float
    current_state: int
    active_errors: int
    disarm_reason: int


class ODriveUsb:
    def __init__(self, device: Any, config: ODriveConfig) -> None:
        self._device = device
        self._axis = device.axis0
        self._config = config

    @classmethod
    async def connect(cls, config: ODriveConfig) -> "ODriveUsb":
        try:
            import odrive
        except ImportError as error:
            raise RuntimeError(
                "ODrive package is unavailable; install this project first"
            ) from error

        kwargs: dict[str, Any] = {"interfaces": ["usb"]}
        if config.serial_number:
            kwargs["serial_number"] = config.serial_number
        device = await odrive.find_async(**kwargs)
        return cls(device, config)

    async def _timed(self, awaitable: Any) -> Any:
        return await asyncio.wait_for(
            awaitable, timeout=self._config.usb_operation_timeout_s
        )

    async def read_feedback(self) -> DriveFeedback:
        values = await self._timed(
            asyncio.gather(
                self._axis.pos_estimate.read(),
                self._axis.vel_estimate.read(),
                self._axis.current_state.read(),
                self._axis.active_errors.read(),
                self._axis.disarm_reason.read(),
            )
        )
        return DriveFeedback(
            position_turns=float(values[0]),
            velocity_turns_s=float(values[1]),
            current_state=int(values[2]),
            active_errors=int(values[3]),
            disarm_reason=int(values[4]),
        )

    async def arm_torque_control(self) -> None:
        from odrive.enums import AxisState, ControlMode, InputMode
        from odrive.utils import request_state

        await self._timed(self._device.clear_errors())
        await self._timed(
            asyncio.gather(
                self._axis.controller.config.control_mode.write(
                    ControlMode.TORQUE_CONTROL
                ),
                self._axis.controller.config.input_mode.write(InputMode.PASSTHROUGH),
                self._axis.config.watchdog_timeout.write(
                    self._config.watchdog_timeout_s
                ),
                self._axis.controller.input_torque.write(0.0),
            )
        )
        await self._timed(self._axis.config.enable_watchdog.write(True))
        await self._timed(self._axis.watchdog_feed())
        await self._timed(request_state(self._axis, AxisState.CLOSED_LOOP_CONTROL))

    async def write_torque(self, torque_nm: float) -> None:
        await self._timed(
            asyncio.gather(
                self._axis.controller.input_torque.write(float(torque_nm)),
                self._axis.watchdog_feed(),
            )
        )

    async def safe_idle(self) -> None:
        from odrive.enums import AxisState
        from odrive.utils import request_state

        # Each operation is independent so a failed zero-torque write does not
        # prevent the IDLE request. The hardware watchdog is the final fallback.
        try:
            await self._timed(self._axis.controller.input_torque.write(0.0))
        except Exception:
            pass
        idle_confirmed = False
        try:
            await self._timed(request_state(self._axis, AxisState.IDLE))
            state = await self._timed(self._axis.current_state.read())
            idle_confirmed = int(state) == int(AxisState.IDLE)
        except Exception:
            pass
        # Never disable the watchdog unless the IDLE transition succeeded. If
        # USB is degraded, leaving it enabled is what eventually stops the axis.
        if idle_confirmed:
            try:
                await self._timed(self._axis.config.enable_watchdog.write(False))
            except Exception:
                pass

    @staticmethod
    def closed_loop_state_value() -> int:
        from odrive.enums import AxisState

        return int(AxisState.CLOSED_LOOP_CONTROL)
