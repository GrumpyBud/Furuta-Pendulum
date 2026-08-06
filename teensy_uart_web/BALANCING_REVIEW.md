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

The old gains `{1.8, -18, 1.25, -2.2}` were arbitrary placeholders. Worse,
the old parser required positive arm gains and negative pendulum gains. That sign
pattern makes the nominal linearized plant unstable. The parser now applies
absolute magnitude limits and the UI preserves the signs of a derived gain
profile.

### Nominal linear model and LQR

The corrected model uses:

```text
m = 0.159 kg                 r = 0.1815 m
l = 0.05379 m                Jp = 0.001141 kg m^2
h = m*l = 0.00855261 kg m    c = r*h = 0.00155230 kg m^2
M11 upright yaw = 0.00561 kg m^2
```

With pendulum positive chosen so a positive logical arm acceleration initially
produces a positive pendulum acceleration, the upright inertia matrix is:

```text
M = [[ 0.005610, -0.0015523],
     [-0.0015523, 0.001141 ]]
```

For `x = [theta, alpha, theta_dot, alpha_dot]`, torque input in N m, and zero
provisional viscous damping, the resulting continuous matrices are:

```text
A = [[0,  0,          1, 0],
     [0,  0,          0, 1],
     [0, 32.61908,     0, 0],
     [0,117.88520,     0, 0]]

B = [[  0       ],
     [  0       ],
     [285.86613 ],
     [388.91291 ]]
```

The plant was zero-order-hold discretized at `0.005 s`. The LQR uses transparent
Bryson-style design scales of `1 rad` arm angle, `0.15 rad` pendulum error,
`3 rad/s` arm rate, `5 rad/s` pendulum rate, and `0.10 N m` command. Equivalently,
before the common `dt` factor:

```text
Q = diag(1, 44.4444, 0.111111, 0.04)
R = 100
```

The nominal discrete gain is:

```text
K = [-0.09140, +1.44432, -0.06921, +0.13886]
tau = -K*x
```

The original first-trial profile was `0.65*K`, or approximately
`[-0.05941, +0.93881, -0.04498, +0.09026]`. After guarded hardware tuning, the
firmware default is `[-0.07000, +1.60000, -0.06920, +0.09000]`; the web page
also retains the conservative, exact nominal, and bounded `1.15*K` profiles.
A sweep over upright yaw inertia
from `0.00510` to `0.00620 kg m^2` leaves the nominal linear poles in the stable
half-plane, but that is a robustness check—not proof of stability on hardware.
The design can be reproduced with `tools/design_balance_lqr.py`.

The original implementation review also augmented the sampled plant with its
then-current 35 Hz pendulum-rate filter and, separately, a pessimistic full one-sample
`5 ms` actuator delay. For all three UI profiles and both endpoints of the yaw
inertia interval, the largest discrete pole magnitude remained below `0.990`.
A sampled linear simulation including the `0.1225 N m` tuning clamp and
`8 N m/s` slew limit decayed throughout the allowed `+/-0.14 rad`, `+/-1 rad/s`
start box without crossing the `0.32 rad` abort or predicted arm-travel limit.
This checks the code/model interaction near upright; it does not model backlash,
flex, motor current-loop dynamics, nonlinear motion, or sensor mounting error.
Three subsequent hands-off hardware trials reached the full 8 s commissioning
window. Their rapid torque reversals motivated reducing the filter to 25 Hz;
that hardware-tuned configuration must pass the new 20 s guarded window before
automatic swing-up is unlocked.

Runtime gain commands are defense-in-depth checked twice: every magnitude must
remain below a hard absolute cap, and each signed term must be between 50% and
150% of the corresponding reviewed LQR term. A serial command cannot restore the
old sign pattern or submit an opposite-sign controller while tuning is unlocked.

`kControlDirectionVerified` remains `false`, so powered upright tuning is still
compile-time locked. The final motor-disabled test must establish the coupling
sign expected by this model; flipping gain signs is not an acceptable substitute.
Automatic swing-up has a second, independent `kAutomaticSwingUpEnabled` lock and
must remain disabled until repeated upright trials and their telemetry are
reviewed.

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

Because the continuous energy term is exactly zero when the pendulum is perfectly motionless at the bottom, automatic run begins with a small positive/negative bounded arm nudge. This removes the mathematical deadlock without requiring encoder noise. The nudge is still real motion and begins immediately after confirmed arming. The energy gain and startup nudge remain empirical; the separate automatic-swing-up interlock prevents them from running merely because upright tuning has been unlocked.

The swing-up torque is separately clamped below the normal maximum. Arm centring was added because damping alone does not prevent slow arm drift into a travel stop.

## Switching and saturation

`SWING_UP` changes to `BALANCE` only when both the absolute upright angle and pendulum speed are inside the catch limits. `BALANCE` returns to `SWING_UP` at a wider angle, giving hysteresis and avoiding rapid mode chatter.

Every controller path reaches a final torque clamp and slew limiter. Tuning has a lower clamp. Saturation means the linear closed-loop poles alone are not enough to prove stability; nonlinear simulation and logged hardware trials must include the same clamps, slew rate, ODrive torque accuracy, and bus/current limiting.

After the `18 A` ODrive hard limit was reported, the old `0.75 N m` firmware
clamp was found to correspond to `24.49 A` at the measured torque constant. The
commissioning clamps are now defined in current units and converted to torque:
`10.0 A` overall, `6.5 A` swing-up, and `4.0 A` tuning. The resulting torque
limits are approximately `0.3063`, `0.1991`, and `0.1225 N m`, respectively.

## Timing and transport review

ODrive UART ASCII is request/response rather than cyclic. At 115200 baud, a 200 Hz loop is close enough to the wire-time limit that it must be measured on the real ODrive firmware. The implementation:

- adds and verifies the ASCII protocol checksum;
- rejects `nan` and `inf` feedback at the UART parser boundary;
- accepts no feedback after the response deadline;
- normally requests arm position/rate every 5 ms;
- periodically replaces one feedback request with an active-error query;
- faults feedback older than 15 ms;
- derives pendulum velocity from the actual elapsed encoder sample time rather than blindly assuming 5 ms;
- records current and worst control-work time in every telemetry row;
- faults repeated missed deadlines.

The arm handshake writes, then reads back, `enable_watchdog = 1`, torque
`control_mode = 1`, and passthrough `input_mode = 1`; a successful byte-level
UART write is no longer treated as proof that ODrive accepted those safety-
critical modes. The official ASCII `c` torque command also feeds the watchdog.

The AS5048A manufacturer register map was checked during this review. Its
Diagnostics + AGC register is `0x3FFD`; an earlier draft incorrectly used
`0x3FFC`, which would have prevented a healthy encoder status. Native tests now
pin the register address, SPI parity, OCF/COF flags, and the bit-10 weak/bit-11
strong magnet labels.

The original draft captured `now` before the blocking feedback request but updated the feedback timestamp after it, which could make unsigned age subtraction wrap and immediately fault. The health check now samples `micros()` after I/O.

Before motion, `max_loop_us` should remain comfortably below the 5000 us period through minutes of hand testing. If it approaches the period, do not weaken the deadline safety check. Improve the UART link/baud and re-characterize the discrete controller, or use CAN.

## Stop and fault-path review

All deliberate disarms and software faults send zero torque and request ODrive `IDLE`. Every motor-active mode requires focused-tab Spacebar dead-man messages at 150 ms intervals; firmware rejects a missing message after 450 ms. A detected key release or focus loss also sends an immediate explicit release command. During active control, torque commands feed a 50 ms ODrive watchdog. If Teensy-to-ODrive UART fails, the software may be unable to deliver zero/idle, so the already-enabled ODrive watchdog is the fallback. A browser key is not safety-rated; a real power disconnect remains necessary for faults that affect the computer, both processors, the drive power stage, or software assumptions.

Active control faults on:

- missing or stale focused-tab Spacebar dead-man messages;
- bad AS5048A magnet diagnostics or repeated SPI errors;
- invalid/stale/checksum-failing UART feedback;
- ODrive active errors;
- non-finite state;
- predicted arm travel stopping margin, arm speed, or pendulum speed limit;
- repeated control deadline misses;
- tuning twenty-second timeout or exit from its narrow upright region.

Boot requests zero torque/idle and starts disarmed. Recoverable motion-envelope,
catch-region, timeout, deadline, and dead-man faults preserve the saved zero
because neither absolute encoder reference changed. Encoder corruption,
non-finite sensor data, an ODrive active error, or lost ODrive feedback
invalidate zero so a potentially stale reference cannot be reused.

## Verification performed without hardware

- native tests pass for state-feedback order, angle-wrap boundaries, torque slew limiting, predictive travel, gain-profile enforcement, dead-man timeout boundaries including timer rollover, AS5048A address/diagnostics/parity, the documented ODrive checksum example, checksum rejection, finite feedback parsing, and non-finite feedback rejection;
- firmware compiles and links for Teensy 4.1 with the pinned Teensy PlatformIO platform;
- JavaScript parses successfully in Node and the Python server byte-compiles;
- the dashboard has a synthetic `?demo=1` stream for parser/render checks.

## Hardware validation still required

No repository-only test can establish motor phase/encoder sign, mechanical direction, UART noise margin, actual torque constant, loop latency on the installed ODrive firmware, safe regenerative limits, or closed-loop stability. The ODrive commissioning settings are now reported as `10 A` motor soft, `18 A` motor hard, finite `+25 A`/`-5 A` DC-current trips, `max_regen_current = 0 A`, and voltage-feedback braking disabled. Its inactive `51 V`/`53 V` voltage-feedback ramp must not be enabled on a bus that trips at `25.5 V`. The `2.4 rad` arm limit now includes an `80 ms` outward-motion projection, leaving more coasting margin before the requested physical `+/-180 degree` envelope; real hard stops are still required. Do not claim “working balance” until all of these have been observed on the guarded, current-limited mechanism and the downloaded logs reviewed.

A minimum acceptance record should show:

- correct arm and pendulum position/velocity signs by hand;
- zero growing SPI/UART error counts through full-speed hand motion;
- control work comfortably under 5 ms, with no deadline faults;
- ODrive enters idle within its watchdog interval when UART TX is disconnected during a tiny constrained torque test;
- every software limit, Spacebar release/focus-loss path, and motor-power disconnect tested independently at low energy;
- bounded tuning trials that decay rather than grow after small upright disturbances;
- catch/drop transitions with torque, travel, bus voltage, and current inside limits.
