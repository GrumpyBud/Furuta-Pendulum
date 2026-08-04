from pathlib import Path
import tempfile
import unittest

from furuta_pi.config import load_config


class ConfigTests(unittest.TestCase):
    def test_defaults_load_from_empty_toml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("", encoding="utf-8")
            config = load_config(path)
        self.assertEqual(config.hardware.encoder_spi_bus, 0)
        self.assertEqual(config.control.control_hz, 200.0)

    def test_unknown_setting_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[hardware]\ntypo_pin = 12\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown hardware setting"):
                load_config(path)

    def test_invalid_spi_speed_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[hardware]\nencoder_spi_max_speed_hz = 0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "must be positive"):
                load_config(path)


if __name__ == "__main__":
    unittest.main()
