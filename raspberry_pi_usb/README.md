# Raspberry Pi 5 + ODrive S1 over USB

This version runs the Furuta controller as a Python process on 64-bit Raspberry Pi OS. The Pi talks to the ODrive S1 through ODrive's native USB protocol, reads the free-pendulum quadrature encoder through Pi GPIO, and reads a normally-closed software safety loop through another GPIO.

This is intended for guarded bench bring-up while a CAN interface is unavailable. It is not equivalent to the legacy Teensy controller:

- Raspberry Pi OS is not hard real-time. The process asks for 200 Hz, but scheduling and USB timing can jitter.
- GPIO Zero decodes encoder edges in Linux userspace and can lose steps under CPU load or at high encoder rates.
- ODrive's documentation recommends CAN over USB for runtime operation.
- The program uses the ODrive hardware watchdog, starts disarmed, clamps torque, checks travel/speed/state, and requests `IDLE` on every fault. Those are useful layers, not a safety-rated system.

Do not attempt swing-up or balance until you have measured the achieved loop timing and verified that the Pi never loses encoder counts through the full expected speed range. A small RP2040/RP2350 encoder front end or the original Teensy is a better long-term way to acquire the pendulum encoder deterministically.

## Required hardware

- Raspberry Pi 5 running current 64-bit Raspberry Pi OS
- ODrive S1 already commissioned and calibrated for its motor/encoder
- **USB isolator between the Pi and ODrive**; ODrive says USB and DC power may only be used together through an isolator
- Incremental quadrature encoder with 3.3 V-compatible A/B outputs
- Normally-closed maintained switch for the software safety loop
- Separate hard-wired E-stop/contactor that removes motor power
- Physical arm stops, guard, current-limited power, and suitable regenerative-energy handling

Never apply 5 V to a Pi GPIO. If the encoder has 5 V, open-collector, or differential outputs, use the correct receiver/level-shifter rather than connecting it directly.

## Default wiring

The configuration uses BCM GPIO numbers:

| Signal | BCM GPIO | Pi header pin | Other end |
|---|---:|---:|---|
| Pendulum encoder A | 17 | 11 | encoder A |
| Pendulum encoder B | 27 | 13 | encoder B |
| Software safety loop | 22 | 15 | normally-closed switch to GND |
| Signal ground | — | 6 | encoder common and switch |
| Encoder power, only if appropriate | — | 1 | 3.3 V encoder supply |
| Motor command/feedback | USB | USB port | USB isolator, then ODrive S1 USB-C |

The GPIO safety loop is active-low with an internal pull-up: a closed switch reads safe, and an open switch or broken wire reads unsafe. It must not be the only E-stop.

## Pi installation

From this directory:

```sh
sudo apt update
sudo apt install python3-venv python3-gpiozero python3-lgpio curl
sudo usermod -a -G gpio "$USER"
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
cp config.example.toml config.toml
```

Log out and back in after adding the `gpio` group. ODrive's official Linux setup also requires its USB udev rule:

```sh
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
```

Connect only the ODrive you intend to control and confirm native USB access:

```sh
odrivetool
```

Close `odrivetool` and the ODrive Web GUI before running the controller. Only one program can claim the native USB interface at a time.

## Configure before running

Copy and edit `config.toml`. At minimum, verify:

- the three BCM GPIO numbers;
- `pendulum_steps_per_revolution` by rotating the pendulum encoder exactly one mechanical revolution;
- both direction signs;
- motor-to-arm gear ratio;
- measured pendulum mass, centre-of-mass length, and inertia;
- conservative torque, speed, travel, ODrive current, DC-current, voltage, and braking limits;
- balance gains derived for the measured mechanism and actual loop period.

GPIO Zero reports one step for a complete quadrature sequence. The old Teensy `Encoder` library used x4 edge counts, so an old value of 8192 counts/revolution usually starts as 2048 steps/revolution here. Verify this on your exact encoder instead of assuming the ratio.

The runtime sets torque-control/passthrough mode and enables a 100 ms ODrive watchdog in volatile configuration. It does not save persistent ODrive configuration. Commission and calibrate the S1 separately with the matching ODrive GUI/docs.

## Test and run

The pure logic tests run without Pi GPIO or an ODrive:

```sh
PYTHONPATH=src python -m unittest discover -s tests -v
```

Run the controller from an interactive terminal:

```sh
source .venv/bin/activate
furuta-pi --config config.toml
```

Commands are `zero`, `arm`, `disarm`, `status`, `help`, and `quit`.

Safe initial sequence:

1. Install stops and guard, open the hardware motor-power E-stop, and leave the mechanism disconnected or unable to drive the arm.
2. Start the program. It must report `DISARMED`.
3. Let the pendulum hang straight down, centre the arm, and send `zero`.
4. Send `status`. Down should read close to `-3.1416`; upright should read close to zero. Rotate each direction by hand and verify both angle and rate signs.
5. Turn the free encoder exactly one revolution and verify the displayed angle returns to the starting value without accumulating an offset. Repeat quickly enough to cover the expected operating speed.
6. Observe the controller under representative Pi CPU/USB load. Any deadline fault or lost encoder count means this architecture is unsuitable at that rate.
7. Only after all checks pass, use a guarded, current-limited, low-torque test with a person at the hardware power E-stop. Send `arm`; use `disarm` for a controlled stop.

After any fault, correct the cause and send `disarm` to acknowledge it before arming again. If USB disconnects, writes fail or the ODrive stops receiving watchdog feeds; the ODrive should enter `IDLE`. A hard-wired power interrupt remains necessary because software and USB can fail in ways no process can handle.

## Why native USB instead of `/dev/ttyACM0`?

The ODrive exposes both a CDC serial interface and a native packet interface over USB. This version uses the official async Python API on the native interface so position/velocity/state reads and torque/watchdog writes are typed and can be issued concurrently. The serial ASCII protocol would add parsing and does not make Linux scheduling deterministic.
