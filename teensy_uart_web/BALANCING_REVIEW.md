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

The hardware coupling direction has now been verified and
`kControlDirectionVerified` is `true`. Guarded swing-up commissioning has its
own `kSwingTuningEnabled` gate; unrestricted automatic swing-up remains behind
the independent `kAutomaticSwingUpEnabled` lock.

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

The first hardware energy trials exposed a limitation in applying that equation
without regard to available arm travel. In the 2026-08-07 run, 664 consecutive
`SWING_UP` samples reached only `1.957 rad` (`112.1 degrees`) from upright before
the arm walked from center to `-1.55 rad` and the pendulum lost energy. The
command was already at the `0.450 N m` ceiling for `30.9%` of those samples, so
raising the ceiling was not the missing behavior. The revised law keeps full
energy pumping near center and whenever the energy torque points back toward
center. Only an outward-pointing energy term fades linearly from full at
`0.65 rad` to zero at `1.10 rad`; arm-centering and arm-rate damping remain
active throughout. A replay on that failed trajectory reduced mean outward
torque beyond the guard from `0.077 N m` to `0.015 N m`, while the stronger
default increased mean absolute swing command from `0.256 N m` to `0.359 N m`.
Replay is a control-logic regression check, not a prediction of the changed
closed-loop trajectory, so the physical guarded trial remains authoritative.

The hardware-informed default is now `k_E = 2.0`, `k_d = 0.030`, and
`k_c = 0.180`, with a `0.220 N m` startup nudge and the unchanged `0.450 N m`
ceiling. The travel guard begins far inside both the `2.4 rad` predictive
software stop and the requested `+/-pi` mechanical envelope.

Because the continuous energy term is exactly zero when the pendulum is perfectly motionless at the bottom, the energy phase begins with one small positive bounded arm nudge, then immediately hands control to the energy law. This removes the mathematical deadlock without relying on encoder noise or an opposing kick that could remove the energy just added.

Before that energy phase, the controller performs two explicit preparation states using ODrive's native position loop. `CENTERING` selects `POSITION_CONTROL` plus `POS_FILTER`, seeds the setpoint at the measured position, and then targets the saved arm zero. The configuration is read back before motion: filter bandwidth `20 1/s`, velocity limit `0.5 turn/s`, position gain `20`, velocity gain `0.167`, velocity-integrator gain `0.333`, velocity limiting enabled, and command torque limited to `0.300 N m`. ODrive retains and filters that target internally; `CENTERING` and `SETTLING` feed its watchdog every 20 ms with the short `u 0` command instead of repeatedly transmitting the longer position command. These ODrive-managed states request arm feedback at 100 Hz, allow up to 9 ms for a reply, and stop after three consecutive misses or 35 ms without a valid reply—still before the 50 ms hardware watchdog. A failed feedback transaction remains immediately due, and successful arming restarts the slower property-health timer, preventing a property query and retry delay from consuming the initial feedback-freshness window. Active and inactive feedback polling are mutually exclusive; an active tick whose 100 Hz request is not due performs no background 4.5 ms transaction. The time-critical Teensy torque loop keeps its original 200 Hz, 4.5 ms transaction schedule and 15 ms freshness limit. The arm must remain within `0.020 rad` at less than `0.050 rad/s`, and the pendulum within `0.040 rad` of hanging at less than `0.120 rad/s`, for `2.5 s` continuously. Settling uses separate low-pass rate estimates: 5 Hz for the ODrive arm velocity and 3 Hz for the pendulum encoder. Hardware logs showed the arm confined to `0.00122 rad` peak-to-peak while its instantaneous velocity estimate spiked to `0.0822 rad/s`; filtering prevents those measurement spikes and stationary AS5048A count jitter from resetting the timer, while tests verify sustained arm motion and the measured approximately 1.4 Hz pendulum oscillation remain detectable. The original estimates remain unchanged for balance control. Any real gate excursion resets the timer. Centering has a 12-second timeout and settling has a 20-second timeout. Once settled, the code changes input mode to passthrough, sends zero torque, and reads back torque mode `1` and input mode `1`; swing-up cannot begin if that handoff fails.

The guarded request still has a broad pre-arm envelope: the stopped arm must be within `1.75 rad` of saved center and the pendulum generally downward within `0.80 rad`; rate gates reject an already fast-moving mechanism. Guarded swing-up has a final 15 A-equivalent `0.4594 N m` clamp below the reported 18 A ODrive hard maximum, while the user-selectable swing ceiling stops at `0.450 N m`. Before closed-loop position preparation, firmware writes a session-only `15 A` motor `current_soft_max`, then reads back that value, `current_hard_max`, and `motor.effective_current_lim`; it refuses motion if the dynamic limit is below the `14.69 A` required by the user ceiling. The balance catch immediately returns to the lower 10 A-equivalent `0.3063 N m` commissioning clamp, and upright tuning remains independently limited to 4 A-equivalent `0.1225 N m`. Runtime energy gain, arm damping, arm centring, startup nudge, and swing torque ceiling are nonnegative and bounded in both the browser and firmware. All preparation and active phases require the focused Spacebar dead-man. Unrestricted automatic mode remains disabled.

## Switching and saturation

`SWING_UP` changes to `BALANCE` only when both the absolute upright angle and pendulum speed are inside the catch limits. `BALANCE` returns to `SWING_UP` at a wider angle, giving hysteresis and avoiding rapid mode chatter.

Every controller path reaches a final torque clamp and slew limiter. Tuning has a lower clamp. Saturation means the linear closed-loop poles alone are not enough to prove stability; nonlinear simulation and logged hardware trials must include the same clamps, slew rate, ODrive torque accuracy, and bus/current limiting. The 2026-08-07 trace also showed that the old request and the transmitted slew-limited torque differed by only `0.004 N m` on average, ruling out the common slew limiter as the dominant cause of that specific plateau; it therefore remains unchanged for both swing and balance.

After the `18 A` ODrive hard limit was reported, the old `0.75 N m` firmware
clamp was found to correspond to `24.49 A` at the measured torque constant. The
commissioning clamps are now defined in current units and converted to torque:
`10.0 A` for balance, `15.0 A` for swing-up (including guarded trials), and
`4.0 A` for upright tuning. The resulting torque limits are approximately
`0.3063`, `0.4594`, and `0.1225 N m`, respectively. Automatic centering remains
at `0.300 N m`; the user-selectable swing ceiling stops at `0.450 N m`, and
centering retains its `0.5 turn/s` speed ceiling. The higher swing limit only
exists physically when ODrive's motor-current soft maximum is at least `15 A`.

## Timing and transport review

ODrive UART ASCII is request/response rather than cyclic. At 115200 baud, a 200 Hz loop is close enough to the wire-time limit that it must be measured on the real ODrive firmware. The implementation:

- adds and verifies the ASCII protocol checksum;
- rejects `nan` and `inf` feedback at the UART parser boundary;
- accepts no feedback after the response deadline;
- normally requests arm position/rate every 5 ms;
- tolerates one missed cyclic-feedback reply only while the last valid sample is
  younger than 15 ms, skips motor-command calculation from that stale sample,
  and faults on the second consecutive miss;
- periodically replaces one feedback request with an active-error query;
- never sends a filtered-position command on that same health-query tick,
  retries an isolated missed property response after 10 ms, and requires three
  consecutive property-query misses before stopping;
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
- upright-tuning twenty-second timeout or exit from its narrow region;
- automatic-centering 12-second or strict-settling 20-second timeout;
- guarded swing-up twenty-second timeout.

Boot requests zero torque/idle and starts disarmed. Recoverable motion-envelope,
catch-region, timeout, deadline, and dead-man faults preserve the saved zero
because neither absolute encoder reference changed. Encoder corruption,
non-finite sensor data, an ODrive active error, or lost ODrive cyclic feedback
invalidate zero so a potentially stale reference cannot be reused. A missed
health-property query alone does not prove reference loss: cyclic feedback is
required between rapid retries, and even three property-only misses stop motion
while preserving zero. If the link itself is gone, the next 5 ms feedback
request faults immediately and invalidates zero.

## Verification performed without hardware

- native tests pass for state-feedback order, angle-wrap boundaries, torque slew limiting, predictive travel, gain-profile enforcement, dead-man timeout boundaries including timer rollover, active/inactive UART poll exclusion, AS5048A address/diagnostics/parity, the documented ODrive checksum example, checksum rejection, finite feedback parsing, and non-finite feedback rejection;
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
