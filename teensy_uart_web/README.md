# Teensy 4.1 + ODrive S1 UART Furuta controller

This is the non-CAN version of the Furuta pendulum controller. A Teensy 4.1 reads an **AS5048A** absolute magnetic encoder over SPI, exchanges checksummed ASCII messages with an ODrive S1 over its isolated **UART**, runs the control loop, and streams telemetry over its USB cable. A local browser page provides guided setup, motor-disabled testing, bounded tuning, logging, and run controls.

“UART” is the name for the simple TX/RX wire connection requested here. This implementation assumes the encoder is an **AS5048A-TS_EK_AB**, based on the existing code and the likely part-name typo “RS4850A.” If the encoder is actually an RS-485 model, do not connect or power it according to this guide—the electrical interface and firmware are different.

> **Experimental machinery warning:** This code is not a safety-rated control system and has not been validated on your particular mechanism. A Furuta pendulum can move suddenly, hit hard, and regenerate energy. Install a physical guard and travel stops, use conservative ODrive current/torque/bus limits, and use a hard-wired E-stop or contactor that removes motor power. The browser, Teensy pin 6, UART, and ODrive watchdog are additional layers—not replacements for removing power.

## What this version includes

- no CAN hardware or CAN library;
- crossed, checksummed ODrive UART at 115200 baud on Teensy `Serial1`;
- AS5048A SPI parity, protocol-error, magnet-strength, and AGC checks;
- boot-to-idle, explicit hanging-down zero, explicit arming confirmation, and fault acknowledgement;
- ODrive watchdog plus browser keep-alive, stale-feedback, deadline, travel, velocity, torque, and torque-slew limits;
- a motor-disabled `TEST` mode with live values and downloadable CSV;
- a separate low-torque `TUNING` mode that works only while its browser button is held, aborts outside the upright region, and stops after eight seconds;
- temporary, range-checked gain changes that revert on reboot;
- swing-up startup nudge, arm centring, energy shaping, and hysteretic balance catch/drop logic.

The web server runs on the computer connected to the Teensy. The Teensy 4.1 does not provide this webpage directly; doing that would require the optional Teensy Ethernet hardware and a different network/security design.

## Wiring—all power off first

Teensy 4.1 pins are **not 5 V tolerant**. Power off the ODrive DC bus, auxiliary supply, Teensy USB, and every external supply before changing wires.

### ODrive S1 UART

| Teensy 4.1 | ODrive S1 isolated I/O | Purpose |
|---|---|---|
| pin `0` / `RX1` | `UART TX` / isolated `G06` output | ODrive sends to Teensy |
| pin `1` / `TX1` | `UART RX` / isolated `G07` input | Teensy sends to ODrive |
| `3.3V` | `V+ ISO` / `ISO VDD` | powers ODrive's isolated UART side at safe 3.3 V logic |
| `GND` | `GND ISO` | isolated UART reference |

TX and RX are intentionally crossed. Do not connect the ODrive isolated UART at 5 V when it is wired directly to the Teensy. “G06/G07” are ODrive names and are unrelated to Teensy pin 6.

### AS5048A SPI adapter

| AS5048A adapter label | Teensy 4.1 |
|---|---|
| `GND` | `GND` |
| `A2/MISO` | pin `12` / MISO |
| `A1/MOSI` | pin `11` / MOSI |
| `SCL/SCK` | pin `13` / SCK |
| `SDA/CSn` | pin `10` / chip select |
| `PWM` | leave disconnected |
| `3.3V` | Teensy `3.3V` |
| `5V` / main supply | also Teensy `3.3V` |

This is the AS5048A's documented 3.3 V supply mode: both adapter supply pins go to 3.3 V. Never connect that combined supply to `VIN`, `VUSB`, or 5 V. Use short SPI wires away from motor phase and DC-bus cables. Use a centred, diametrically magnetized two-pole magnet at the adapter manufacturer's recommended gap.

### Software safety loop

Connect a normally-closed maintained switch between Teensy pin `6` and Teensy `GND`. A closed healthy loop reads low; an open switch or broken wire faults active control. Use a separate hard-wired motor-power E-stop as described above.

## Configure the ODrive before connecting UART

Calibrate the motor and its encoder with the official ODrive GUI first. Confirm the motor encoder is reliable and test torque control at a tiny torque with the mechanism disconnected. Configure conservative motor current, DC current, velocity, bus overvoltage, brake-resistor, and thermistor limits for the real hardware.

The S1 UART is disabled by default. With the Teensy disconnected from the ODrive UART, use the ODrive GUI or `odrivetool` for the installed firmware version and set:

```python
from odrive.enums import GpioMode

odrv0.config.uart_a_baudrate = 115200
odrv0.config.enable_uart_a = True
odrv0.config.gpio6_mode = GpioMode.UART_A
odrv0.config.gpio7_mode = GpioMode.UART_A
odrv0.save_configuration()
```

Then power-cycle the ODrive. These names match current ODrive S1 documentation; use the documentation matching the exact installed firmware if its GUI exposes different names. The firmware sets torque/passthrough mode, a 50 ms ODrive watchdog, and closed-loop state at each arm request. It does not save those runtime changes to ODrive flash.

Official references: [ODrive UART interface](https://docs.odriverobotics.com/v/latest/manual/uart.html), [Arduino/Teensy UART guide](https://docs.odriverobotics.com/v/latest/guides/arduino-uart-guide.html), [ASCII protocol](https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html), and [S1 datasheet](https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html).

## Measure and edit the mechanism configuration

Open `include/config.hpp` and review every setting before allowing motion. At minimum:

- `kMotorTurnsToArmRadians`: include the motor-to-arm gear ratio; direct drive is `2*pi`;
- `kMotorDirection`: `1` or `-1`, chosen so the dashboard arm coordinate increases in the intended positive direction;
- `kPendulumDirection`: `1` or `-1`, chosen so the pendulum coordinate increases in the intended positive direction;
- pendulum mass, pivot-to-centre-of-mass length, and pivot inertia;
- arm travel and both velocity limits;
- ODrive and firmware torque/current limits;
- the controller gains, after identifying or modelling the actual mechanism.

The torque output is transformed by `kMotorDirection` as well as the arm measurement. Do not “fix” a wiring/sign problem by randomly flipping gain signs.

The included gains are placeholders carried from the earlier implementation. They are not claimed to stabilize an arbitrary Furuta pendulum. See `BALANCING_REVIEW.md` for the exact equations, assumptions, review findings, and the required hardware validation sequence.

## Build, test, and upload

Install [PlatformIO](https://platformio.org/), then from this directory run:

```sh
pio test -e native
pio run -e teensy41
pio run -e teensy41 -t upload
```

This implementation has no third-party Arduino library dependency beyond the Teensy framework's SPI library. If upload waits, briefly press the Teensy's Program button.

## Start the local dashboard

With the latest firmware uploaded and the Teensy connected by USB:

```sh
python3 web/serve.py
```

Open [http://localhost:8765](http://localhost:8765) in a current desktop Chrome or Edge browser, then click **Connect Teensy** and choose its USB serial port. Web Serial requires the first connection to come from a user click. The server binds only to `127.0.0.1`, has no Python package dependencies, and the page loads no cloud resources. Use `?demo=1` to preview without hardware.

Keep the control page open and visible during automatic motion. The firmware requires a browser keep-alive during normal run and a faster dead-man keep-alive during tuning. Closing, disconnecting, hiding/throttling, or crashing the control page causes a software fault; the ODrive watchdog is the independent short fallback.

## Beginner-safe commissioning sequence

1. Mechanically disconnect the motor or remove motor power. Install guards and physical stops now—not later.
2. Power only logic, open the dashboard, and connect. The controller must report `DISARMED`.
3. On **Setup**, confirm clean encoder diagnostics and ODrive UART feedback. Keep the software safety loop closed only while intentionally testing it.
4. Enter **Sensor test**. Move the mechanism slowly by hand through its expected range. Verify smooth angles, correct signs, zero SPI errors, and a worst loop-work time comfortably below 5 ms.
5. Centre the rotary arm, let the pendulum hang straight down and motionless, then save zero. The dashboard should show about `-180°` hanging and `0°` upright.
6. Disconnect motor power again before editing any sign or scale constant. Rebuild, upload, and repeat the sensor test until every measurement is correct.
7. Calibrate and test the ODrive separately with the linkage disconnected and a tiny current/torque limit.
8. Only after expert review of the model/gains, install the guard, clear the full envelope, enable current-limited motor power, and use **Tune**. Hold the pendulum upright; the test runs only while the button remains held.
9. Change one gain only by 5–10%, repeat a short test, and download the CSV. A fault invalidates zero; correct the cause, press **DISARM / STOP**, and zero again.
10. Use **Run** only after bounded upright trials are consistently stable. Arming begins the startup nudge and swing-up immediately.

## Why tuning is guided rather than automatic

Blind automatic gain search on an unknown unstable mechanism can command unsafe motion and learn from collisions or saturation instead of plant dynamics. This UI therefore offers plain-language gain controls, bounded values, low tuning torque, upright-only start/abort regions, a hold-to-run dead man, an eight-second limit, logging, and non-persistent trials. A defensible automatic design would first require measured arm inertia, motor torque constant/dynamics, pendulum inertia/friction, sample/transport delay, a validated nonlinear model, and a constrained system-identification experiment.

## Troubleshooting

- **ODrive UART never turns green:** confirm UART is enabled and saved, baud is 115200, TX/RX are crossed, and `V+ ISO` is powered from Teensy 3.3 V with `GND ISO` connected.
- **UART faults during motion:** shorten/separate UART wiring, verify the ODrive is not emitting unrelated debug text, and inspect `loop_us`/`max_loop_us` in the downloaded CSV. UART ASCII is more interference-sensitive and timing-limited than CAN.
- **Encoder says weak/strong magnet:** correct magnet centring and gap before testing motion.
- **SPI errors increase:** shorten SPI wiring, separate it from motor cables, or conservatively lower `kEncoderSpiHz`.
- **Hanging angle is not near -180° after zero:** the zero was saved in the wrong pose or the sensor is not reporting one clean mechanical revolution.
- **A sign is backward:** change the corresponding direction constant with all motor power off, rebuild, and repeat sensor testing.
- **Tuning refuses to start:** all five setup checks must pass, the controller must be `DISARMED` or `TEST`, and the pendulum must be within about 8° of upright and moving slower than 1 rad/s.
- **Fault after browser/tab change:** expected fail-safe behavior; acknowledge with disarm and save zero again.

UART is convenient and removes the CAN transceiver, but ODrive explicitly describes its Arduino UART library/protocol path as hobby-oriented and recommends CAN for professional/noisy environments. Do not treat fewer wires as greater fault tolerance.
