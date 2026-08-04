"""Raspberry Pi 5 AS5048A SPI and safety-loop inputs."""

from __future__ import annotations

from .config import HardwareConfig
from .as5048a import AS5048AEncoder


class PiInputs:
    """Read an active-low safety loop and an AS5048A absolute encoder."""

    def __init__(self, config: HardwareConfig) -> None:
        try:
            from gpiozero import Button
            from gpiozero.pins.lgpio import LGPIOFactory
        except ImportError as error:
            raise RuntimeError(
                "GPIO support is unavailable; install python3-gpiozero and python3-lgpio"
            ) from error

        self._factory = LGPIOFactory(chip=config.gpio_chip)
        try:
            self._encoder = AS5048AEncoder(
                bus=config.encoder_spi_bus,
                device=config.encoder_spi_device,
                max_speed_hz=config.encoder_spi_max_speed_hz,
                diagnostic_interval_reads=config.encoder_diagnostic_interval_reads,
            )
        except Exception:
            self._factory.close()
            raise
        try:
            self._estop = Button(
                config.estop_bcm,
                pull_up=True,
                bounce_time=0.005,
                pin_factory=self._factory,
            )
        except Exception:
            self._encoder.close()
            self._factory.close()
            raise

    @property
    def pendulum_count(self) -> int:
        return self._encoder.read_count()

    @property
    def estop_closed(self) -> bool:
        # A normally-closed switch pulls the input low, which GPIO Zero reports
        # as a pressed button. A broken wire therefore reads unsafe.
        return bool(self._estop.is_pressed)

    def close(self) -> None:
        self._estop.close()
        self._encoder.close()
        self._factory.close()
