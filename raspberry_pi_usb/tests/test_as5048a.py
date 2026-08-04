import unittest

from furuta_pi.as5048a import (
    AS5048AEncoder,
    AS5048AMagnetError,
    AS5048AParityError,
    AS5048AProtocolError,
    ERROR_FLAG,
    NOP_COMMAND,
    REGISTER_ANGLE,
    REGISTER_CLEAR_ERROR,
    REGISTER_DIAGNOSTICS,
    build_read_command,
    diagnostic_problem,
    parse_response,
    with_even_parity,
)


class FakeSpi:
    def __init__(self, responses: list[int]) -> None:
        self.responses = iter(responses)
        self.sent: list[int] = []
        self.mode = 0
        self.max_speed_hz = 0
        self.bits_per_word = 0
        self.closed = False

    def xfer2(self, data: list[int]) -> list[int]:
        self.sent.append((data[0] << 8) | data[1])
        response = next(self.responses)
        return [(response >> 8) & 0xFF, response & 0xFF]

    def close(self) -> None:
        self.closed = True


class AS5048AProtocolTests(unittest.TestCase):
    def test_read_angle_command_is_ffff_with_even_parity(self) -> None:
        self.assertEqual(build_read_command(REGISTER_ANGLE), 0xFFFF)

    def test_parse_response_returns_all_14_angle_bits(self) -> None:
        response = with_even_parity(0x2A55)
        self.assertEqual(parse_response(response), 0x2A55)

    def test_parse_response_rejects_bad_parity(self) -> None:
        with self.assertRaises(AS5048AParityError):
            parse_response(with_even_parity(0x1234) ^ 0x0001)

    def test_parse_response_rejects_error_flag(self) -> None:
        with self.assertRaises(AS5048AProtocolError):
            parse_response(with_even_parity(ERROR_FLAG | 0x0123))

    def test_diagnostic_flags(self) -> None:
        self.assertIsNone(diagnostic_problem(1 << 8))
        self.assertEqual(
            diagnostic_problem((1 << 8) | (1 << 10)),
            "magnetic field too weak",
        )

    def test_read_count_uses_pipelined_angle_then_diagnostics(self) -> None:
        angle = 0x2345
        healthy_diagnostics = 1 << 8
        spi = FakeSpi(
            [
                0,
                with_even_parity(angle),
                0,
                with_even_parity(healthy_diagnostics),
            ]
        )
        encoder = AS5048AEncoder(spi=spi, diagnostic_interval_reads=20)
        self.assertEqual(encoder.read_count(), angle)
        self.assertEqual(
            spi.sent,
            [
                build_read_command(REGISTER_ANGLE),
                NOP_COMMAND,
                build_read_command(REGISTER_DIAGNOSTICS),
                NOP_COMMAND,
            ],
        )

    def test_bad_magnet_diagnostic_is_rejected(self) -> None:
        spi = FakeSpi(
            [
                0,
                with_even_parity(0x1234),
                0,
                with_even_parity((1 << 8) | (1 << 11)),
            ]
        )
        encoder = AS5048AEncoder(spi=spi, diagnostic_interval_reads=20)
        with self.assertRaises(AS5048AMagnetError):
            encoder.read_count()

    def test_protocol_error_is_cleared_before_raising(self) -> None:
        spi = FakeSpi(
            [
                0,
                with_even_parity(ERROR_FLAG | 0x0042),
                0,
                0,
            ]
        )
        encoder = AS5048AEncoder(spi=spi)
        with self.assertRaises(AS5048AProtocolError):
            encoder.read_register(REGISTER_ANGLE)
        self.assertEqual(spi.sent[2], build_read_command(REGISTER_CLEAR_ERROR))


if __name__ == "__main__":
    unittest.main()
