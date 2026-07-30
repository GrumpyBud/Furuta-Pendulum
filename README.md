# Furuta pendulum controller

Firmware for a **Teensy 4.1**, an **ODrive S1**, a motor/arm encoder managed by the ODrive, and a quadrature encoder on the free pendulum. It uses energy shaping to swing up, then switches to full-state feedback near upright. The firmware starts disarmed on every boot and faults open on stale CAN data, an ODrive error, excessive travel/speed, an open E-stop circuit, or a missed control deadline.

> **This is experimental machinery, not a finished safety system.** A Furuta pendulum can move suddenly and stores enough energy to injure someone. Use physical arm stops, guards, current-limited bench power, an accessible hardware E-stop that removes motor power, and safety glasses. First tests should use low torque limits with the linkage disconnected. Software cannot replace a power contactor.

## Electrical connections

| Signal | Teensy 4.1 | Other end |
|---|---:|---|
| CAN TX | pin 22 / CTX1 | 3.3 V CAN-transceiver TXD |
| CAN RX | pin 23 / CRX1 | 3.3 V CAN-transceiver RXD |
| Pendulum encoder A/B | pins 2 / 3 | encoder A/B (3.3 V logic) |
| Safety loop | pin 6 and GND | normally-closed maintained switch |
| Ground | GND | transceiver, encoder, and ODrive signal ground |

The Teensy does **not** contain a CAN physical-layer transceiver. Put a 3.3 V-compatible transceiver between it and CAN-H/CAN-L. Terminate the two ends of the CAN bus with 120 ohms; do not add termination at every device. Check the encoder output voltage before connecting it to the Teensy, whose inputs are not 5 V tolerant.

The pin-6 loop is only a low-voltage request to the firmware. Use a separate, hard-wired E-stop to interrupt motor power or the drive-enable chain. Opening either the switch or its wire causes a firmware fault.

## ODrive preparation

This code deliberately does not write persistent ODrive configuration. Configure and validate the S1 using the matching ODrive documentation and tools before allowing the arm to move:

1. Set CAN node ID `0` and CAN baud rate `250000`, or change both values in `include/config.hpp`.
2. Calibrate the motor and its encoder; confirm positive motor turns agree with the positive arm-angle convention. Save the ODrive configuration.
3. Configure the CAN heartbeat at 10 Hz or faster and encoder-estimates cyclic message at **500 Hz or faster**. The controller rejects feedback older than 20 ms.
4. Configure conservative ODrive current, velocity, DC-current, and braking limits. The firmware's torque clamp is an additional limit, not a replacement.
5. Confirm torque-control/passthrough mode works at a tiny torque with the mechanism disconnected.

ODrive firmware releases can change configuration property names. Use the guide belonging to the exact firmware installed on the S1 rather than copying an old command sequence.

## Mechanical measurements and signs

Edit every value in `include/config.hpp` before expecting good control:

- `kPendulumCountsPerRevolution` is the count returned by `Encoder` for one mechanical turn, including x4 quadrature decoding.
- `kMotorTurnsToArmRadians` includes the motor-to-arm gear ratio. It is `2*pi` only for direct drive.
- Reverse `kMotorDirection` or `kPendulumDirection` if the corresponding measurement has the wrong sign; never fix sign mistakes by randomly changing controller gains.
- Measure pendulum mass, pivot-to-centre-of-mass distance, and pivot inertia. A bifilar-pendulum or period measurement is better than a material-density guess.
- The provided feedback gains are safe-shaped placeholders, **not universal gains**. Derive LQR/pole-placement gains from the measured plant and verify the signs with the arm constrained before balancing.

Angle convention is explicit: the `zero` command is issued while the pendulum hangs down; internally, pendulum angle is wrapped to `[-pi, pi)` with zero at upright. Arm angle is zero at the motor estimate captured by the same command.

## Build, test, and upload

Install [PlatformIO](https://platformio.org/), then run:

```sh
pio test -e native
pio run -e teensy41
pio run -e teensy41 -t upload
pio device monitor -b 115200
```

Dependencies are pinned to exact revisions for reproducible public builds. Review and test updates rather than silently following their default branches.

## Operating sequence

1. Raise the mechanism clear of obstructions, install its guard, and keep the hardware motor-power E-stop open.
2. Power logic. The serial console must say `DISARMED`.
3. With motor power still disabled, let the pendulum hang motionless downward and centre the arm. Send `zero`.
4. Send `status`. The pendulum should read close to `-3.1416`; rotate it by hand to upright and confirm it reads close to zero. Confirm both velocity signs and arm-angle sign.
5. Close the firmware safety loop, enable current-limited motor power, clear the area, and send `arm`. Send `disarm` to return the ODrive to idle.
6. After any fault, correct its cause, then send `disarm`; this explicit action is required before arming again.

Commands are newline-terminated: `zero`, `arm`, `disarm`, `status`, and `help`. Status telemetry is also printed once per second. Do not use verbose USB output while diagnosing real-time deadline faults.

## Control structure

At 500 Hz, the Teensy reads the ODrive's arm position/rate and differentiates the pendulum encoder through a 45 Hz first-order low-pass filter. Away from upright, an energy error controller pumps the pendulum while damping arm motion. A low-energy, low-rate catch region switches to state feedback; a wider drop region gives hysteresis and prevents mode chatter. All output paths meet at a final torque saturation.

For serious tuning, log timestamped state and commanded torque to a separate buffered channel, identify friction and inertia, linearize at upright, compute discrete gains for the actual 2 ms sample time, and evaluate stability with actuator saturation and transport delay included. Change one limit or gain at a time and preserve the data behind each change.
