# Teensy 4.1 + ODrive S1 UART Furuta controller

This is the non-CAN version of the Furuta pendulum controller. A Teensy 4.1 reads an **AS5048A** absolute magnetic encoder over SPI, exchanges checksummed ASCII messages with an ODrive S1 over its isolated **UART**, runs the control loop, and streams telemetry over its USB cable. A local browser page provides guided setup, motor-disabled testing, bounded tuning, logging, and run controls.

“UART” is the name for the simple TX/RX wire connection requested here. This implementation assumes the encoder is an **AS5048A-TS_EK_AB**, based on the existing code and the likely part-name typo “RS4850A.” If the encoder is actually an RS-485 model, do not connect or power it according to this guide—the electrical interface and firmware are different.

> **Experimental machinery warning:** This code is not a safety-rated control system and has not been validated on your particular mechanism. A Furuta pendulum can move suddenly, hit hard, and regenerate energy. Install a physical guard and travel stops, use conservative ODrive current/torque/bus limits, and keep a real motor-power disconnect within reach. The browser Spacebar control, UART, and ODrive watchdog are additional software/electronic layers—not replacements for removing power.

## What this version includes

- no CAN hardware or CAN library;
- crossed, checksummed ODrive UART at 115200 baud on Teensy `Serial1`;
- AS5048A SPI parity, protocol-error, magnet-strength, and AGC checks;
- boot-to-idle, explicit hanging-down zero, explicit arming confirmation, and fault acknowledgement;
- ODrive watchdog plus browser keep-alive, stale-feedback, deadline, travel, velocity, torque, and torque-slew limits;
- a motor-disabled `TEST` mode with live values and downloadable CSV;
- a focused-tab Spacebar dead-man required by all motor-active modes;
- a separate low-torque `TUNING` mode that aborts outside the upright region and stops after twenty seconds;
- temporary, range-checked gain changes that revert on reboot;
- a separately gated, 4 A-equivalent guarded swing-up tuner with a 20-second timeout, centered/down start checks, bounded runtime settings, energy shaping, and automatic balance catch;
- an independent lock that still prevents unrestricted automatic swing-up.

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

TX and RX are intentionally crossed. Do not connect the ODrive isolated UART at 5 V when it is wired directly to the Teensy. “G06/G07” are ODrive GPIO names, not Teensy pin numbers.

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

Official encoder reference: [ams OSRAM AS5048A/B datasheet](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf). The firmware uses its `0x3FFD` Diagnostics + AGC register and `0x3FFF` corrected-angle register.

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
- `kPendulumDirection`: `1` or `-1`, chosen by the coupling-sign test below—not by gain trial and error;
- pendulum mass, pivot-to-centre-of-mass length, and pivot inertia;
- arm travel and both velocity limits;
- ODrive and firmware torque/current limits;
- the controller gains, after identifying or modelling the actual mechanism.

The torque output is transformed by `kMotorDirection` as well as the arm measurement. Do not “fix” a wiring/sign problem by randomly flipping gain signs.

The old arbitrary gains were replaced by a 200 Hz discrete LQR derived from the measured mechanism and corrected assembled STEP, then refined through guarded hardware trials. Firmware now starts with the validated `[-0.07000, 1.60000, -0.06920, 0.09000]` profile. The exact matrices, weights, assumptions, and robustness sweep are in [`BALANCING_REVIEW.md`](BALANCING_REVIEW.md).

Current measurements are tracked in [`MECHANISM_DATA.md`](MECHANISM_DATA.md), with the assembled STEP/material calculation in [`CAD_MASS_PROPERTIES.md`](CAD_MASS_PROPERTIES.md). The code records the `0.159 kg` pendulum, `0.1815 m` arm radius, `0.05379 m` COM, `0.001141 kg m^2` pendulum inertia, and `0.00561 kg m^2` complete upright yaw inertia. The hardware coupling sign is verified. Guarded swing-up tuning is enabled independently, while unrestricted automatic swing-up remains locked behind `kAutomaticSwingUpEnabled`.

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

Keep the control page focused and continuously hold **Space** whenever motor motion is intended. On **Tune**, an upright/motionless pendulum starts the low-torque balance trial automatically. On **Run**, a centered arm and hanging/motionless pendulum starts the guarded swing-up trial automatically. No mouse click is required for either commissioning trial. Releasing Space, changing tabs, minimizing the window, disconnecting, or crashing the page stops the browser dead-man messages. The Teensy independently rejects stale messages after 450 ms, and the ODrive watchdog is the shorter motor-command fallback once armed.

The Spacebar control is intentionally ignored while typing in a form field. Click outside a gain input before holding Space. A browser key is not a safety-rated E-stop and cannot protect against a frozen operating system, USB stack, ODrive fault, or power-stage fault.

## Beginner-safe commissioning sequence

1. Mechanically disconnect the motor or remove motor power. Install guards and physical stops now—not later.
2. Power only logic, open the dashboard, and connect. The controller must report `DISARMED`.
3. On **Setup**, confirm clean encoder diagnostics and ODrive UART feedback. Press and hold Space in the focused page and confirm the Spacebar check turns green; release it and confirm the check immediately clears.
   If the UART is online but the ODrive reports a latched error, correct the underlying condition first, then use **Clear ODrive errors**. The button is disabled during every motor-active mode and verifies the live error mask after clearing.
4. Enter **Sensor test**. Move the mechanism slowly by hand through its expected range. Verify smooth angles, correct signs, zero SPI errors, and a worst loop-work time comfortably below 5 ms.
5. Centre the rotary arm, let the pendulum hang straight down and motionless, then save zero. The dashboard should show about `-180°` hanging and `0°` upright.
6. With motor power still disconnected, establish the model coupling sign. Move the arm gently but briskly in the dashboard's positive direction while the pendulum is near upright and free to lag. The initial pendulum angle must move **positive** as its top lags the arm. If it moves negative, change only `kPendulumDirection`, rebuild, and repeat. Once this is repeatable, set `kControlDirectionVerified = true`; do not alter gain signs.
7. Calibrate and test the ODrive separately with the linkage disconnected and a tiny current/torque limit.
8. Only after expert review of the model/gains, install the guard, clear the full envelope, enable current-limited motor power, and use **Tune**. Hold the pendulum upright and then hold Space; the low-torque trial starts automatically. Release Space to stop.
9. Start with **Hardware tuned**, which is also the firmware default. Repeat a guarded test and download the CSV after each profile. Travel, speed, catch-region, timeout, deadline, and dead-man faults preserve the saved zero because neither encoder reference changed. Encoder corruption, non-finite sensor data, ODrive active errors, or loss of ODrive feedback invalidate it. The fault banner states which case occurred. Exact individual gains are exposed only as advanced controls.
10. After bounded upright trials are consistently stable, use **Run** for guarded swing-up tuning. Center the arm, let the pendulum hang motionless, then hold Space. The 4 A-equivalent, 20-second trial starts automatically, attempts the upright catch with the validated balance gains, and logs its swing settings in every CSV row. Full automatic motion remains locked until these trials are reviewed.

## Why tuning is guided rather than automatic

Blind automatic gain search on an unstable mechanism can command unsafe motion and learn from collisions or saturation instead of real dynamics. The UI therefore offers the hardware-tuned default plus three model-based profiles, advanced bounded values, low tuning torque, upright-only start/abort regions, a focused-tab Spacebar dead-man, a twenty-second guarded limit, and logging. Runtime gain edits reset to the validated default after reboot. Hardware trials remain manual and supervised because damping, transport delay, actuator behavior, and model error still have to be validated.

## Troubleshooting

- **ODrive UART never turns green:** confirm UART is enabled and saved, baud is 115200, TX/RX are crossed, and `V+ ISO` is powered from Teensy 3.3 V with `GND ISO` connected.
- **UART faults during motion:** shorten/separate UART wiring, verify the ODrive is not emitting unrelated debug text, and inspect `loop_us`/`max_loop_us` in the downloaded CSV. UART ASCII is more interference-sensitive and timing-limited than CAN.
- **Encoder says weak/strong magnet:** correct magnet centring and gap before testing motion.
- **SPI errors increase:** shorten SPI wiring, separate it from motor cables, or conservatively lower `kEncoderSpiHz`.
- **Hanging angle is not near -180° after zero:** the zero was saved in the wrong pose or the sensor is not reporting one clean mechanical revolution.
- **A sign is backward:** change the corresponding direction constant with all motor power off, rebuild, and repeat the explicit coupling-sign procedure. Do not flip a gain.
- **Tuning refuses to start:** all six setup checks must pass, `kControlDirectionVerified` must be true, Space must remain held in the focused tab, the controller must be `DISARMED` or `TEST`, and the pendulum must be within about 8° of upright and moving slower than 1 rad/s.
- **Full automatic stays disabled:** expected during guarded swing-up commissioning; it has a separate `kAutomaticSwingUpEnabled` interlock.
- **Guarded swing-up will not start:** center the arm within about 20°, let the pendulum hang within about 10° of straight down, hold both nearly motionless, and hold Space on the Run page.
- **Motion stops after releasing Space or changing tabs:** expected dead-man behavior. A successfully delivered release returns to `DISARMED`; a lost keep-alive produces a fault that must be acknowledged, but its saved zero is retained.

UART is convenient and removes the CAN transceiver, but ODrive explicitly describes its Arduino UART library/protocol path as hobby-oriented and recommends CAN for professional/noisy environments. Do not treat fewer wires as greater fault tolerance.
