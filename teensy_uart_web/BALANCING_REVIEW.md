# Balancing and swing-up code review

This review covers the implemented control path in `src/main.cpp`, `include/control_math.hpp`, and `include/config.hpp`. It distinguishes code-level correctness from plant-level stability. The former can be tested without the mechanism; the latter cannot be guaranteed until the actual hardware parameters, signs, delays, actuator limits, and gains are measured and validated.

## Coordinate and torque conventions

The saved zero pose is:

- rotary arm centred, giving arm angle `theta = 0`;
- pendulum hanging downward, which is stored as pendulum angle `alpha = -pi`;
- pendulum upright is `alpha = 0` after wrapping to `[-pi, pi)`.

The state is ordered as:

```text
x = [theta, alpha, theta_dot, alpha_dot]
```

ODrive position/velocity are converted to the logical arm coordinate by:

```text
theta     = (motor_turns - saved_turns) * turns_to_arm_radians * motor_direction
theta_dot = motor_turns_per_second       * turns_to_arm_radians * motor_direction
```

The controller calculates logical torque in that same coordinate. Before UART transmission, torque is multiplied by `motor_direction` to convert back to the ODrive motor coordinate. This output transform is essential when the direction is `-1`; it was missing in the older implementation and was corrected in this version.

## Balance controller

Inside the upright region, the law is full-state feedback:

```text
tau = -(K_theta * theta
      + K_alpha * alpha
      + K_theta_dot * theta_dot
      + K_alpha_dot * alpha_dot)
```

The code and native unit test agree on the state order. Output then passes through the tuning/normal torque clamp and a slew-rate limiter before the direction transform.

The default gain signs assume the repository's chosen positive coordinates. Their magnitudes are placeholders, not a derivation. Stability depends on the complete linearized `A` and `B` matrices, including arm inertia/length, pendulum inertia/mass/COM, motor and transmission dynamics, friction, and the sign of the arm-pendulum coupling. Those values are not all present in this repository, so no honest code review can certify the default gains as stable.

Recommended derivation:

1. measure the full mechanical parameters and motor torque mapping;
2. derive or identify the continuous nonlinear model around upright;
3. linearize it with these exact coordinates and input sign;
4. discretize at the achieved feedback sample time, including measured UART delay;
5. calculate LQR or pole-placement gains with actuator saturation represented;
6. simulate nonlinear swing/catch/drop and limit events;
7. validate signs at extremely low torque with the arm constrained;
8. use the low-torque, upright-only dead-man trials before enabling swing-up.

## Swing-up controller

The pendulum energy expression is:

```text
E = 0.5 * J_p * alpha_dot^2 + m*g*l*(cos(alpha) - 1)
```

This gives desired energy `E = 0` at motionless upright and `E = -2*m*g*l` at motionless downward. The energy-shaping term is:

```text
tau_energy = k_E * (-E) * alpha_dot * cos(alpha)
```

It pumps toward the upright energy and naturally changes phase through the swing. Arm-rate damping and arm-position centring are subtracted:

```text
tau = tau_energy - k_d*theta_dot - k_c*theta
```

Because the continuous energy term is exactly zero when the pendulum is perfectly motionless at the bottom, automatic run begins with a small positive/negative bounded arm nudge. This removes the mathematical deadlock without requiring encoder noise. The nudge is still real motion and begins immediately after confirmed arming.

The swing-up torque is separately clamped below the normal maximum. Arm centring was added because damping alone does not prevent slow arm drift into a travel stop.

## Switching and saturation

`SWING_UP` changes to `BALANCE` only when both the absolute upright angle and pendulum speed are inside the catch limits. `BALANCE` returns to `SWING_UP` at a wider angle, giving hysteresis and avoiding rapid mode chatter.

Every controller path reaches a final torque clamp and slew limiter. Tuning has a lower clamp. Saturation means the linear closed-loop poles alone are not enough to prove stability; nonlinear simulation and logged hardware trials must include the same clamps, slew rate, ODrive torque accuracy, and bus/current limiting.

## Timing and transport review

ODrive UART ASCII is request/response rather than cyclic. At 115200 baud, a 200 Hz loop is close enough to the wire-time limit that it must be measured on the real ODrive firmware. The implementation:

- adds and verifies the ASCII protocol checksum;
- accepts no feedback after the response deadline;
- normally requests arm position/rate every 5 ms;
- periodically replaces one feedback request with an active-error query;
- faults feedback older than 15 ms;
- derives pendulum velocity from the actual elapsed encoder sample time rather than blindly assuming 5 ms;
- records current and worst control-work time in every telemetry row;
- faults repeated missed deadlines.

The original draft captured `now` before the blocking feedback request but updated the feedback timestamp after it, which could make unsigned age subtraction wrap and immediately fault. The health check now samples `micros()` after I/O.

Before motion, `max_loop_us` should remain comfortably below the 5000 us period through minutes of hand testing. If it approaches the period, do not weaken the deadline safety check. Improve the UART link/baud and re-characterize the discrete controller, or use CAN.

## Stop and fault-path review

All deliberate disarms and software faults send zero torque and request ODrive `IDLE`. During active control, torque commands feed a 50 ms ODrive watchdog. Normal run also requires browser keep-alives; tuning uses a tighter dedicated dead-man keep-alive. If Teensy-to-ODrive UART fails, the software may be unable to deliver zero/idle, so the already-enabled ODrive watchdog is the fallback. A hard-wired power interrupt remains necessary for faults that affect both processors, the drive power stage, or software assumptions.

Active control faults on:

- open normally-closed safety loop;
- bad AS5048A magnet diagnostics or repeated SPI errors;
- invalid/stale/checksum-failing UART feedback;
- ODrive active errors;
- non-finite state;
- arm travel, arm speed, or pendulum speed limit;
- repeated control deadline misses;
- browser run keep-alive loss;
- tuning hold loss, eight-second timeout, or exit from its narrow upright region.

Boot requests zero torque/idle and starts disarmed. Faults invalidate the saved zero so a stale reference cannot be reused casually.

## Verification performed without hardware

- native tests pass for state-feedback order, wrap boundaries, slew limiting, AS5048A parity, the documented ODrive checksum example, checksum rejection, and feedback parsing;
- firmware compiles and links for Teensy 4.1 with the pinned Teensy PlatformIO platform;
- JavaScript parses successfully in Node and the Python server byte-compiles;
- the dashboard has a synthetic `?demo=1` stream for parser/render checks.

## Hardware validation still required

No repository-only test can establish motor phase/encoder sign, mechanical direction, UART noise margin, actual torque constant, loop latency on the installed ODrive firmware, safe regenerative limits, or closed-loop stability. Do not claim “working balance” until all of these have been observed on the guarded, current-limited mechanism and the downloaded logs reviewed.

A minimum acceptance record should show:

- correct arm and pendulum position/velocity signs by hand;
- zero growing SPI/UART error counts through full-speed hand motion;
- control work comfortably under 5 ms, with no deadline faults;
- ODrive enters idle within its watchdog interval when UART TX is disconnected during a tiny constrained torque test;
- every software limit and the physical E-stop tested independently at low energy;
- bounded tuning trials that decay rather than grow after small upright disturbances;
- catch/drop transitions with torque, travel, bus voltage, and current inside limits.
