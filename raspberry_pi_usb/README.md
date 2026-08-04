# Raspberry Pi 5 + ODrive S1 over USB

This version runs the Furuta controller as a Python process on 64-bit Raspberry Pi OS. The Pi talks to the ODrive S1 through ODrive's native USB protocol, reads the free pendulum's AS5048A-TS_EK_AB absolute magnetic encoder through SPI, and reads a normally-closed software safety loop through GPIO.

This is intended for guarded bench bring-up while a CAN interface is unavailable. It is not equivalent to the legacy Teensy controller:

- Raspberry Pi OS is not hard real-time. The process asks for 200 Hz, but scheduling and USB timing can jitter.
- The AS5048A is read as a 14-bit absolute angle with SPI parity, protocol-error, and magnetic-field diagnostic checks.
- ODrive's documentation recommends CAN over USB for runtime operation.
- The program uses the ODrive hardware watchdog, starts disarmed, clamps torque, checks travel/speed/state, and requests `IDLE` on every fault. Those are useful layers, not a safety-rated system.

Do not attempt swing-up or balance until you have measured the achieved loop timing and verified clean AS5048A data through the full expected speed range. The original Teensy remains a better long-term hard-real-time controller.

## Required hardware

- Raspberry Pi 5 running current 64-bit Raspberry Pi OS
- ODrive S1 already commissioned and calibrated for its motor/encoder
- **USB isolator between the Pi and ODrive**; ODrive says USB and DC power may only be used together through an isolator
- ams OSRAM AS5048A-TS_EK_AB SPI adapter board and a correctly mounted diametric magnet
- Normally-closed maintained switch for the software safety loop
- Separate hard-wired E-stop/contactor that removes motor power
- Physical arm stops, guard, current-limited power, and suitable regenerative-energy handling

Never apply 5 V to a Pi GPIO. This wiring runs the AS5048A board in 3.3 V mode.

## Default wiring

Use the breakout's silkscreen labels as the primary reference. In 3.3 V mode, the AS5048A datasheet requires its `3.3V` regulator pin to be tied to its supply input, so **both breakout supply pins connect to Pi 3.3 V**. Do not connect the breakout's pin labelled `5V` to Pi 5 V in this configuration.

| Breakout P1 | Breakout label | Raspberry Pi 5 |
|---:|---|---|
| 1 | `GND` | physical pin 6, GND |
| 2 | `A2/MISO` | physical pin 21, GPIO9/SPI0 MISO |
| 3 | `A1/MOSI` | physical pin 19, GPIO10/SPI0 MOSI |
| 4 | `SCL/SCK` | physical pin 23, GPIO11/SPI0 SCLK |
| 5 | `SDA/CSn` | physical pin 24, GPIO8/SPI0 CE0 |
| 6 | `PWM` | leave disconnected |
| 7 | `3.3V` | physical pin 1, 3.3 V |
| 8 | `5V`/supply input | physical pin 1, **3.3 V**, tied to P1-7 |

Additional connections:

| Signal | Raspberry Pi 5 | Other end |
|---|---|---|
| Software safety loop | physical pin 15, GPIO22 | normally-closed switch to GND |
| Motor command/feedback | USB port | USB isolator, then ODrive S1 USB-C |

The GPIO safety loop is active-low with an internal pull-up: a closed switch reads safe, and an open switch or broken wire reads unsafe. It must not be the only E-stop.

## Pi installation

From this directory:

```sh
sudo apt update
sudo apt install python3-venv python3-gpiozero python3-lgpio python3-spidev curl
sudo usermod -a -G gpio,spi "$USER"
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
cp config.example.toml config.toml
```

Enable SPI with `sudo raspi-config`, choose **Interface Options → SPI → Yes**, and reboot. After reboot, this must show the configured device:

```sh
ls -l /dev/spidev0.0
```

Log out and back in after adding the `gpio` and `spi` groups. ODrive's official Linux setup also requires its USB udev rule:

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

- `encoder_spi_bus = 0` and `encoder_spi_device = 0` for `/dev/spidev0.0`;
- the GPIO22 software safety-loop input;
- both direction signs;
- motor-to-arm gear ratio;
- measured pendulum mass, centre-of-mass length, and inertia;
- conservative torque, speed, travel, ODrive current, DC-current, voltage, and braking limits;
- balance gains derived for the measured mechanism and actual loop period.

The AS5048A resolution is fixed at 16,384 counts per revolution. The driver sends the datasheet's pipelined read command, verifies even parity and the SPI error flag on every angle, and checks the offset-compensation, CORDIC-overflow, magnet-too-weak, and magnet-too-strong diagnostics every 20 samples by default.

Mount a diametrically magnetized two-pole magnet concentrically over or under the sensor. The adapter-board manual specifies a 0.5–2 mm air gap and centring within 0.5 mm; use a non-ferromagnetic holder.

The runtime sets torque-control/passthrough mode and enables a 100 ms ODrive watchdog in volatile configuration. It does not save persistent ODrive configuration. Commission and calibrate the S1 separately with the matching ODrive GUI/docs.

## Test and run

The pure logic tests run without Pi GPIO or an ODrive:

```sh
PYTHONPATH=src python -m unittest discover -s tests -v
```

With motor power disabled, test only the AS5048A and safety-loop wiring:

```sh
source .venv/bin/activate
furuta-pi --config config.toml --check-encoder
```

Rotate the magnet slowly. `encoder_raw` must move smoothly through 0–16383 and wrap once per mechanical revolution. The program reports parity, SPI, or magnetic-field diagnostics instead of silently accepting a bad reading. Press Ctrl-C to stop.

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
4. Send `status`. Down should read close to `-3.1416`; upright should read close to zero. Rotate each direction by hand, verify the angle/rate signs, and confirm `encoder_raw` changes smoothly through 0–16383.
5. Turn the pendulum exactly one revolution and verify `encoder_raw` returns to its starting value. Any parity, SPI error, or magnet diagnostic will stop the program or fault an armed controller.
6. Observe the controller under representative Pi CPU/USB load. Any deadline fault means this architecture is unsuitable at that rate.
7. Only after all checks pass, use a guarded, current-limited, low-torque test with a person at the hardware power E-stop. Send `arm`; use `disarm` for a controlled stop.

After any fault, correct the cause and send `disarm` to acknowledge it before arming again. If USB disconnects, writes fail or the ODrive stops receiving watchdog feeds; the ODrive should enter `IDLE`. A hard-wired power interrupt remains necessary because software and USB can fail in ways no process can handle.

## Why native USB instead of `/dev/ttyACM0`?

The ODrive exposes both a CDC serial interface and a native packet interface over USB. This version uses the official async Python API on the native interface so position/velocity/state reads and torque/watchdog writes are typed and can be issued concurrently. The serial ASCII protocol would add parsing and does not make Linux scheduling deterministic.
