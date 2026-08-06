# Mechanism data and tuning worksheet

This file is the source of truth for the physical Furuta pendulum. Do not set
`kMechanismSetupComplete` to `true` until every required measurement is filled
in, the balance gains have been calculated from the resulting model, and the
low-energy validation plan has been reviewed.

STEP volume, material, printed-mass, and arm-inertia calculations are recorded
in [`CAD_MASS_PROPERTIES.md`](CAD_MASS_PROPERTIES.md).

## Confirmed measurements

| Item | Value | Status and notes |
|---|---:|---|
| Transmission | 1 motor rev : 1 arm rev | Confirmed direct drive |
| Motor revs per arm rev | `1.0` | Entered in `include/config.hpp` |
| Measured pendulum-side lump | `0.159 kg` | Includes the rod, printed block, horizontal shaft, shaft collars, set screws, and one complete bearing; the other complete bearing was excluded |
| Pendulum small-oscillation period | `0.7327 s` | Average of three five-cycle trials: `3.61`, `3.70`, and `3.68 s` |
| Pendulum rod | `8 mm` diameter, `200 mm` long, stainless steel | Calculated mass `0.08042 kg` using nominal `8000 kg/m^3` density |
| Pivot-to-rod offset | `0.00635 m` (`1/4 in`) | Horizontal shaft centre to nearest rod end |
| Pivot connection and bearing lump | 3D-printed 90-degree block, set screws, horizontal shaft, shaft collars, and one complete bearing | Remaining mass `0.07858 kg`, provisionally concentrated at the pivot axis |
| Calculated pendulum COM distance | `0.05379 m` | Provisional component model; near-pivot hardware location is the main uncertainty |
| Pendulum inertia about pivot | `0.001141 kg m^2` | Derived from measured `0.7327 s` period and provisional COM |
| Motor axis to pendulum pivot | `0.3215 m` | Magnitude of `-12.657480 in`; physical length is stored positive |
| Motor | ODrive D5065, `270 KV`, 7 pole pairs | Model identified |
| Motor torque constant | `0.0306296 N m/A` | Matches approximately `8.27 / 270` |
| Battery | Turnigy Graphene Panther, `6S` LiPo, `5.0 Ah`, `100C`, `22.2 V` nominal, XT90 | Pack-label discharge rating confirmed; allowed charge C still required |
| DC breaker | `30 A` | DC-bus protection; not a motor phase-current limit |
| Brake resistor | `2 ohm`, `50 W` | ODrive enable is `true` and configured resistance is `2 ohm`; wiring still requires physical verification |
| ODrive motor current soft max | `10 A` | Confirmed commissioning setting |
| ODrive motor current hard max | `18 A` | Confirmed current setting |
| ODrive velocity limit | `1.5 turn/s` | `9.4248 rad/s` at the direct-drive arm |
| Torque-mode velocity limiter | Enabled; `vel_gain = 0.167` | Confirmed current settings |
| ODrive DC current trip limits | `+25 A`, `-5 A` | Confirmed finite commissioning settings |
| ODrive bus-voltage trips | `19.8 V` low, `25.5 V` high | Exactly `3.30 V/cell` and `4.25 V/cell` for 6S |
| ODrive maximum supply regen current | `0 A` | Global main setting; brake resistor begins shunting all estimated regen |
| Voltage-feedback braking | Disabled | Leave disabled during commissioning; current-based resistor braking remains active |
| Voltage-feedback ramp | `51 V` start, `53 V` end | Inactive while disabled; invalid for 6S if enabled because both exceed the `25.5 V` trip |

For the bearing uncertainty, the target mass includes the pendulum rod, weights,
fasteners, shaft, encoder magnet, collars, spacers, and bearing inner races that
rotate with the pendulum. Fixed outer races and housings belong to the rotary-arm
assembly. If the two bearings are identical, including one complete bearing while
excluding the other may be a reasonable first approximation, but record the
bearing part number or total bearing mass before assigning an uncertainty.

## Required next measurements

All dimensions should be in metres, masses in kilograms, time in seconds, inertia
in kg m^2, torque in N m, and angular velocity in rad/s.

### 1. Pendulum centre of mass

- Complete moving pendulum assembly mass: `0.159 kg`
- Distance from horizontal pendulum pivot axis to combined COM: `0.05379 m`

The pivot is the centreline of the horizontal shaft through the bearings—the
axis around which the pendulum swings. It is not the motor's vertical shaft. For
this nearly uniform rod, a component calculation is easier than balancing the
assembled L shape. The rod's own centre of mass is exactly halfway along its
length: `100 mm` from either end for a `200 mm` rod. If the rod begins a distance
`d` below the horizontal pivot axis, its centre is `d + 100 mm` from that axis.

Using nominal stainless density `8000 kg/m^3`, the rod mass is `0.08042 kg` and
its centre is `0.10635 m` from the pivot. The remaining `0.07858 kg` is the
printed block, set screws, horizontal shaft, collars, and the one included full
bearing. Treating that remainder as concentrated on the pivot axis gives:

```text
l = (0.08042 * 0.10635 + 0.07858 * 0) / 0.159
  = 0.05379 m
```

This is a documented first-model assumption, not a claim that the hardware has
zero size. If the remainder's true COM is between `0` and `10 mm` from the axis,
the combined COM is approximately `53.8` to `58.7 mm`. The measured period and
rod-only geometric inertia agree within about `3.2%`, supporting this model for
conservative initial controller calculations.

A complete bearing includes a fixed outer race, so the `0.159 kg` is a convenient
dynamic bookkeeping lump rather than a literal claim that every gram rotates
about the pendulum shaft. Because the included bearing is centred on the pivot,
it contributes essentially no gravitational first moment. Its horizontal motion
with the rotary arm is represented by retaining the lumped mass at the arm end.
When arm inertia is assembled, this `0.07858 kg` pivot lump must not be added a
second time.

### 2. Pendulum inertia about its pivot

- Complete oscillations per run: `5`
- Measured elapsed times: `3.61 s`, `3.70 s`, `3.68 s`
- Mean elapsed time for five periods: `3.6633 s`
- Measured period: `T = 0.7327 s`
- Run-to-run sample standard deviation: `0.0473 s` per five-period run, or
  approximately `0.0095 s` in the calculated period

The repeat resolves the original alternating-endpoint count. The damping that
stops the pendulum after roughly twelve swings should be recorded and later
estimated, but the early small-amplitude cycles provide a usable period.

As a geometry sanity check, an ideal uniform `0.200 m` rod pivoted at one end has
`T = 2*pi*sqrt(2*L/(3*g)) = 0.7325 s`, almost exactly the measured `0.7327 s`.
This supports both the full-cycle count and the approximately uniform-rod model;
the remaining mass allocation is still needed for the controller model.

With the rotary arm held stationary, displace the pendulum less than about 10
degrees and time complete back-and-forth cycles. Calculate:

```text
J_p = m * g * l * (T / (2*pi))^2
```

Using `m = 0.159 kg`, `l = 0.05379 m`, `T = 0.7327 s`, and
`g = 9.80665 m/s^2` gives `J_p = 0.001141 kg m^2`. This becomes
`kPendulumInertiaKgM2`; its uncertainty follows the near-pivot COM assumption.

### 3. Rotary-arm geometry and inertia

- Vertical motor-axis centre to the pendulum rod's pivot/attachment centre:
  `0.1815 m`; the assembled STEP has `0.3215 m` with a `280 mm` extrusion, so
  replacing that extrusion with the real `140 mm` member moves the complete
  outer assembly inward by `140 mm`
- Rotary arm: `140 mm` 2020 aluminum extrusion, CAD-derived mass `69.23 g`
- Aluminum hub: CAD-derived mass `16.69 g`
- PETG motor adapter and far-end bearing/encoder supports: CAD-derived nominal
  printed mass `49.77 g`, range `36.19–63.34 g`
- Two bearings: CAD steel estimate `11.38 g` total
- Encoder/wiring and unmodelled screw allowance: `11 g`
- Raw CAD motor-side rigid assembly mass: `0.1654 kg`, range
  `0.1518–0.1790 kg`; allocation-adjusted nominal `0.1597 kg`, range
  `0.1462–0.1733 kg`, after removing the one bearing-equivalent already in the
  pendulum lump
- Corrected complete upright yaw inertia about the motor axis:
  `0.00561 kg m^2`, engineering range `0.00510–0.00620 kg m^2`; this is
  assembled-body `M(0,0)` and already includes the pendulum-side yaw mass
- Motor rotor inertia from datasheet, if available: **TODO**

The confirmed centre-to-centre distance becomes `kArmPivotRadiusM`. Yaw inertia
was calculated component by component with the parallel-axis theorem:

```text
J_about_motor_axis = J_at_component_COM + mass * distance_to_axis^2
```

For a uniform radial bar of length `r`, `J = mass * r^2 / 3`. For direct drive,
motor rotor inertia is added without a gear-ratio multiplier. The complete
upright total becomes `kUprightArmAxisInertiaKgM2`. Do not add a separate
`0.159*r^2` term to that value; doing so would move near-axis shaft and bearing
mass out to the rod attachment and double-count it.

The single assembled `Main Assembly.step` confirms that the 78 mm horizontal
pendulum shaft is **parallel** to the radial 2020 arm. This is the expected
Furuta orientation: the pendulum swings in the tangential/vertical plane. An
earlier perpendicular-axis warning came from misreading separately exported
bodies and has been withdrawn.

### 4. Motor and ODrive torque data

- Motor make and model: `ODrive D5065`
- Motor KV in rpm/V: `270 KV`
- ODrive `axis0.config.motor.torque_constant`: `0.0306296 N m/A`
- ODrive `axis0.config.motor.current_soft_max`: `10 A`
- ODrive `axis0.config.motor.current_hard_max`: `18 A`
- ODrive `axis0.controller.config.vel_limit`: `1.5 turn/s`
- ODrive `axis0.controller.config.enable_torque_mode_vel_limit`: `true`
- ODrive `axis0.controller.config.vel_gain`: `0.167`
- Battery: Turnigy Graphene Panther `6S` LiPo, `5.0 Ah`, `22.2 V` nominal,
  `25.2 V` fully charged, XT90
- Battery discharge C printed on this pack: `100C`
- Battery maximum allowed charge C: **TODO**
- DC breaker: `30 A`
- ODrive `config.dc_max_positive_current`: `25 A`
- ODrive `config.dc_max_negative_current`: `-5 A`
- ODrive bus undervoltage/overvoltage trips: `19.8 V` / `25.5 V`
- Brake resistor: `2 ohm`, `50 W`; ODrive enable `true`, configured resistance `2 ohm`
- ODrive global `config.max_regen_current`: `0 A`
- ODrive `enable_dc_bus_voltage_feedback`: `false`
- ODrive voltage-feedback ramp start/end: `51 V` / `53 V` (inactive while disabled)

The pack's advertised discharge rating will be far above this rig's needs, but
it must not be used as the commissioning limit. The `30 A` breaker and wiring
are the tighter constraints. Start with a finite positive DC trip below the
breaker rating (provisionally `25 A`) and keep the motor soft/hard limits at
about `10 A` / `18 A`. A breaker has a time/current trip curve, so the final DC
limit must be checked against its exact datasheet and the wire/connector ratings.

The firmware command clamps now use the same commissioning envelope explicitly:

| Mode | Phase-current equivalent | Torque clamp at `0.0306296 N m/A` |
|---|---:|---:|
| Absolute maximum | `10.0 A` | `0.3063 N m` |
| Swing-up (including guarded trials) | `8.0 A` | `0.2450 N m` |
| Upright tuning trial | `4.0 A` | `0.1225 N m` |
| Automatic arm centering | `5.88 A` | `0.1800 N m` |

The previous `0.75 N m` overall clamp would have requested `24.49 A`, above
the newly reported `18 A` ODrive hard trip, so it was removed. These smaller
values are commissioning limits only; model-based simulation and guarded tests
must justify any later increase.

The `100C` discharge label means a theoretical `500 A` pack rating; it does not
override the `30 A` breaker, wiring, ODrive, or commissioning limits. The finite
`-5 A` trip is deliberately conservative because discharge C does not specify
safe charging current.

`max_regen_current` is correctly located in the ODrive's global/main settings,
not inside `brake_resistor0`. At `0 A`, the brake-resistor controller attempts to
shunt all estimated regenerative current. The separate voltage-feedback ramp is
a bus-voltage backup. The reported `51 V` / `53 V` values can never activate in
this setup because the ODrive faults at `25.5 V` first. Voltage-feedback braking
is confirmed disabled and should remain disabled during commissioning; the
separate current-based brake-resistor control remains active at
`max_regen_current = 0 A`.

At `25.2 V`, a `2 ohm` resistor would draw `12.6 A` and dissipate `317.5 W` at
100% duty. Its `50 W` marking is therefore a continuous/average constraint, not
permission for sustained full-duty braking. Keep the enclosure clear of it and
verify its pulse-energy specification or temperature during bounded tests.

The expected torque constant is approximately `8.27 / KV` N m/A. Record the
actual ODrive value rather than silently assuming the motor label is exact.

### 5. Limits, signs, and measured timing

- Physical arm travel left/right from centre: **TODO**
- Desired software travel limit: currently `2.4 rad` (`137.5 degrees`)
- Verified `kMotorDirection`: `+1`; whole-arm clockwise motion increases the dashboard arm coordinate
- Verified `kPendulumDirection`: `-1`; raw encoder value decreases for pendulum clockwise motion, and the observed coupling is arm CW -> pendulum CW / arm CCW -> pendulum CCW
- Typical dashboard `loop_us`: **TODO**
- Worst dashboard `max_loop_us` over several minutes: **TODO**
- Arm and pendulum friction/decay observations: **TODO**

## Controller calculation status

- Nonlinear model populated: **NO**
- Upright linear model checked: **NO**
- 200 Hz discrete model with measured delay: **NO**
- LQR weights selected and documented: **NO**
- Torque saturation and travel limits simulated: **NO**
- Low-torque sign test completed: **NO**
- Balance gains accepted: **NO**
- Swing-up constants accepted: **NO**
- `kMechanismSetupComplete`: `false`
