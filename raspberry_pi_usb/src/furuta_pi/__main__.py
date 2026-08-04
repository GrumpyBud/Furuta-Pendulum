"""Command-line entry point."""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import signal

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
    return parser.parse_args()


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
        asyncio.run(async_main(args.config))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        raise SystemExit(f"fatal: {error}") from error


if __name__ == "__main__":
    main()
