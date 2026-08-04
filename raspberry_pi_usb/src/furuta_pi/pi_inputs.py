"""Raspberry Pi 5 GPIO inputs, loaded only on the target machine."""

from __future__ import annotations

from .config import HardwareConfig


class PiInputs:
    """Read an active-low safety loop and an incremental quadrature encoder."""

    def __init__(self, config: HardwareConfig) -> None:
        try:
            from gpiozero import Button, RotaryEncoder
            from gpiozero.pins.lgpio import LGPIOFactory
        except ImportError as error:
            raise RuntimeError(
                "GPIO support is unavailable; install python3-gpiozero and python3-lgpio"
            ) from error

        self._factory = LGPIOFactory(chip=config.gpio_chip)
        self._encoder = RotaryEncoder(
            config.encoder_a_bcm,
            config.encoder_b_bcm,
            max_steps=0,
            wrap=False,
            pin_factory=self._factory,
        )
        self._estop = Button(
            config.estop_bcm,
            pull_up=True,
            bounce_time=0.005,
            pin_factory=self._factory,
        )

    @property
    def pendulum_steps(self) -> int:
        return int(self._encoder.steps)

    @property
    def estop_closed(self) -> bool:
        # A normally-closed switch pulls the input low, which GPIO Zero reports
        # as a pressed button. A broken wire therefore reads unsafe.
        return bool(self._estop.is_pressed)

    def close(self) -> None:
        self._estop.close()
        self._encoder.close()
        self._factory.close()
