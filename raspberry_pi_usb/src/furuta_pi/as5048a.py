"""AS5048A 14-bit absolute magnetic encoder over Raspberry Pi SPI."""

from __future__ import annotations

from typing import Any

COUNTS_PER_REVOLUTION = 1 << 14
READ_FLAG = 1 << 14
PARITY_FLAG = 1 << 15
ERROR_FLAG = 1 << 14
DATA_MASK = COUNTS_PER_REVOLUTION - 1

REGISTER_CLEAR_ERROR = 0x0001
REGISTER_DIAGNOSTICS = 0x3FFC
REGISTER_ANGLE = 0x3FFF
NOP_COMMAND = 0x0000


class AS5048AError(RuntimeError):
    """Base class for encoder communication and diagnostic failures."""


class AS5048AParityError(AS5048AError):
    """The encoder response failed its even-parity check."""


class AS5048AProtocolError(AS5048AError):
    """The encoder reported a previous SPI framing/command error."""


class AS5048AMagnetError(AS5048AError):
    """The encoder diagnostics report an unusable magnetic field."""


def with_even_parity(word: int) -> int:
    """Set bit 15 when needed so the complete 16-bit word has even parity."""
    word &= 0x7FFF
    if word.bit_count() & 1:
        word |= PARITY_FLAG
    return word


def build_read_command(register: int) -> int:
    if not 0 <= register <= DATA_MASK:
        raise ValueError("AS5048A register must be a 14-bit address")
    return with_even_parity(READ_FLAG | register)


def parse_response(word: int) -> int:
    """Validate an AS5048A read response and return its 14 data bits."""
    word &= 0xFFFF
    if word.bit_count() & 1:
        raise AS5048AParityError(f"AS5048A response parity error: 0x{word:04X}")
    if word & ERROR_FLAG:
        raise AS5048AProtocolError(f"AS5048A SPI error flag: 0x{word:04X}")
    return word & DATA_MASK


def diagnostic_problem(diagnostics: int) -> str | None:
    """Decode the AS5048A diagnostic/AGC register into a fault reason."""
    if not diagnostics & (1 << 8):
        return "offset compensation not finished"
    if diagnostics & (1 << 9):
        return "CORDIC overflow; magnet alignment or field is invalid"
    if diagnostics & (1 << 10):
        return "magnetic field too weak"
    if diagnostics & (1 << 11):
        return "magnetic field too strong"
    return None


class AS5048AEncoder:
    """Read absolute angle and diagnostics from one AS5048A on SPI."""

    def __init__(
        self,
        bus: int = 0,
        device: int = 0,
        max_speed_hz: int = 1_000_000,
        diagnostic_interval_reads: int = 20,
        *,
        spi: Any | None = None,
    ) -> None:
        if diagnostic_interval_reads <= 0:
            raise ValueError("diagnostic_interval_reads must be positive")
        if spi is None:
            try:
                import spidev
            except ImportError as error:
                raise RuntimeError(
                    "SPI support is unavailable; install python3-spidev"
                ) from error
            spi = spidev.SpiDev()
            spi.open(bus, device)
        self._spi = spi
        self._spi.mode = 0b01
        self._spi.max_speed_hz = max_speed_hz
        self._spi.bits_per_word = 8
        self._diagnostic_interval_reads = diagnostic_interval_reads
        self._reads_until_diagnostics = 0

    def _transfer_word(self, word: int) -> int:
        received = self._spi.xfer2([(word >> 8) & 0xFF, word & 0xFF])
        if len(received) != 2:
            raise AS5048AProtocolError(
                f"AS5048A SPI returned {len(received)} bytes instead of 2"
            )
        return (int(received[0]) << 8) | int(received[1])

    def _clear_error(self) -> None:
        self._transfer_word(build_read_command(REGISTER_CLEAR_ERROR))
        self._transfer_word(NOP_COMMAND)

    def read_register(self, register: int) -> int:
        # The AS5048A response is pipelined: the requested register arrives in
        # the response to the following 16-bit transaction.
        self._transfer_word(build_read_command(register))
        response = self._transfer_word(NOP_COMMAND)
        try:
            return parse_response(response)
        except AS5048AProtocolError:
            self._clear_error()
            raise

    def check_diagnostics(self) -> None:
        diagnostics = self.read_register(REGISTER_DIAGNOSTICS)
        problem = diagnostic_problem(diagnostics)
        if problem:
            raise AS5048AMagnetError(f"AS5048A {problem}")

    def read_count(self) -> int:
        angle = self.read_register(REGISTER_ANGLE)
        if self._reads_until_diagnostics <= 0:
            self.check_diagnostics()
            self._reads_until_diagnostics = self._diagnostic_interval_reads
        self._reads_until_diagnostics -= 1
        return angle

    def close(self) -> None:
        self._spi.close()
