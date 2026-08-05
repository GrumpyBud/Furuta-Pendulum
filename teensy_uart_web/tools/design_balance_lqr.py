#!/usr/bin/env python3
"""Reproduce the model-based Furuta upright gains stored in config.hpp."""

from __future__ import annotations

import argparse
import sys

try:
    import numpy as np
    from scipy.linalg import eigvals, solve_discrete_are
    from scipy.signal import cont2discrete
except ImportError as error:
    raise SystemExit(
        "This design-only tool needs NumPy and SciPy: "
        "python3 -m pip install numpy scipy"
    ) from error


def plant(mass: float, com: float, radius: float, pendulum_inertia: float,
          yaw_inertia: float) -> tuple[np.ndarray, np.ndarray]:
    gravity = 9.80665
    first_moment = mass * com
    coupling = radius * first_moment
    inertia = np.array(
        [[yaw_inertia, -coupling], [-coupling, pendulum_inertia]],
        dtype=float,
    )
    if np.linalg.det(inertia) <= 0.0:
        raise ValueError("inertia matrix is not positive definite")

    inverse = np.linalg.inv(inertia)
    a = np.zeros((4, 4))
    a[0, 2] = 1.0
    a[1, 3] = 1.0
    a[2:4, 1] = inverse @ np.array([0.0, first_moment * gravity])
    b = np.zeros((4, 1))
    b[2:4, 0] = inverse[:, 0]
    return a, b


def discrete_lqr(a: np.ndarray, b: np.ndarray, period: float) -> tuple:
    ad, bd, _, _, _ = cont2discrete(
        (a, b, np.eye(4), np.zeros((4, 1))), period
    )
    # Bryson-style scales: 1 rad arm, 0.15 rad pendulum, 3 and 5 rad/s,
    # and 0.10 N m input. Multiplying Q and R by dt approximates the
    # continuous-time cost integral and leaves their relative weighting clear.
    q = np.diag([1.0, 1.0 / 0.15**2, 1.0 / 3.0**2, 1.0 / 5.0**2]) * period
    r = np.array([[1.0 / 0.10**2]]) * period
    p = solve_discrete_are(ad, bd, q, r)
    gain = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    return ad, bd, gain


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mass", type=float, default=0.159)
    parser.add_argument("--com", type=float, default=0.05379)
    parser.add_argument("--radius", type=float, default=0.1815)
    parser.add_argument("--pendulum-inertia", type=float, default=0.001141)
    parser.add_argument("--yaw-inertia", type=float, default=0.00561)
    parser.add_argument("--period", type=float, default=0.005)
    args = parser.parse_args()

    a, b = plant(args.mass, args.com, args.radius,
                 args.pendulum_inertia, args.yaw_inertia)
    ad, bd, gain = discrete_lqr(a, b, args.period)

    np.set_printoptions(precision=8, suppress=True)
    print("State: [arm, pendulum_upright, arm_rate, pendulum_rate]")
    print("Sign: positive arm acceleration must initially produce positive pendulum acceleration")
    print("Continuous A:\n", a)
    print("Continuous B:\n", b)
    print("Discrete LQR K (torque = -K*x):\n", gain[0])
    for scale in (0.65, 1.0, 1.15):
        poles = eigvals(ad - bd @ (scale * gain))
        equivalent = np.log(poles) / args.period
        print(f"{scale:>4.2f}x gains: {scale * gain[0]}")
        print(f"     equivalent poles: {equivalent}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
