"""Command-line entry point."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import signal
import time

from .as5048a import COUNTS_PER_REVOLUTION
from .config import load_config
from .controller import FurutaController, install_console
from .odrive_usb import ODriveUsb
from .pi_inputs import PiInputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config.toml"),
        help="TOML configuration path (default: config.toml)",
    )
    parser.add_argument(
        "--check-encoder",
        action="store_true",
        help="read and display the AS5048A without connecting to the ODrive",
    )
    return parser.parse_args()


def check_encoder(config_path: Path) -> None:
    config = load_config(config_path)
    inputs = PiInputs(config.hardware)
    print("AS5048A check mode; rotate the pendulum by hand. Press Ctrl-C to stop.")
    try:
        while True:
            count = inputs.pendulum_count
            degrees = count * 360.0 / COUNTS_PER_REVOLUTION
            safety = "closed" if inputs.estop_closed else "OPEN"
            print(
                f"encoder_raw={count:5d} angle_deg={degrees:8.3f} "
                f"safety_loop={safety}",
                flush=True,
            )
            time.sleep(0.05)
    finally:
        inputs.close()


async def async_main(config_path: Path) -> None:
    config = load_config(config_path)
    inputs = PiInputs(config.hardware)
    drive: ODriveUsb | None = None
    controller: FurutaController | None = None
    try:
        target = config.odrive.serial_number or "the first ODrive"
        print(f"Waiting for {target} on native USB...", flush=True)
        drive = await ODriveUsb.connect(config.odrive)
        controller = FurutaController(config, inputs, drive)
        install_console(controller)
        loop = asyncio.get_running_loop()
        for signal_name in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(signal_name, controller.request_stop)
        await controller.run()
    finally:
        if drive is not None:
            await drive.safe_idle()
        inputs.close()


def main() -> None:
    args = parse_args()
    try:
        if args.check_encoder:
            check_encoder(args.config)
        else:
            asyncio.run(async_main(args.config))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        raise SystemExit(f"fatal: {error}") from error


if __name__ == "__main__":
    main()
