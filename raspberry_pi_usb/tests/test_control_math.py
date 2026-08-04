import math
import unittest

from furuta_pi.control_math import (
    Gains,
    State,
    balance_torque,
    clamp,
    swing_up_torque,
    wrap_angle,
)


class ControlMathTests(unittest.TestCase):
    def test_wrap_angle_boundaries(self) -> None:
        self.assertAlmostEqual(wrap_angle(0.0), 0.0)
        self.assertAlmostEqual(wrap_angle(math.pi), -math.pi)
        self.assertAlmostEqual(wrap_angle(2.0 * math.pi + 0.25), 0.25)

    def test_clamp(self) -> None:
        self.assertEqual(clamp(-2.0, -1.0, 1.0), -1.0)
        self.assertEqual(clamp(0.2, -1.0, 1.0), 0.2)
        self.assertEqual(clamp(2.0, -1.0, 1.0), 1.0)

    def test_balance_feedback_sign_and_sum(self) -> None:
        state = State(1.0, 2.0, 3.0, 4.0)
        gains = Gains(1.0, 10.0, 100.0, 1000.0)
        self.assertEqual(balance_torque(state, gains), -4321.0)

    def test_swing_up_is_zero_at_target_energy(self) -> None:
        upright_at_rest = State(0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(
            swing_up_torque(upright_at_rest, 0.1, 0.2, 0.003, 5.0, 0.1),
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
