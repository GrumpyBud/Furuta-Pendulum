# Teensy 4.1 AS5048A reader

Standalone firmware that reads an `AS5048A-TS_EK_AB` magnetic encoder over SPI and streams CSV over the Teensy's USB serial port. It does not communicate with an ODrive and cannot command a motor.

The output includes the 14-bit absolute and zero-relative angles, degrees, automatic-gain-control value, magnetic diagnostics, and cumulative SPI parity/protocol error counts. It also includes a polished local web dashboard for viewing and zeroing the encoder.

## Power everything off first

Disconnect the ODrive, motor power, Teensy USB cable, and every external supply before wiring. Teensy 4.1 digital pins accept only 0–3.3 V and are not 5 V tolerant.

## Wiring

Use the breakout's printed labels as the primary reference:

| Breakout P1 | Breakout label | Teensy 4.1 |
|---:|---|---|
| 1 | `GND` | `GND` |
| 2 | `A2/MISO` | pin `12` / MISO |
| 3 | `A1/MOSI` | pin `11` / MOSI |
| 4 | `SCL/SCK` | pin `13` / SCK |
| 5 | `SDA/CSn` | pin `10` / chip select |
| 6 | `PWM` | leave disconnected |
| 7 | `3.3V` | Teensy `3.3V` output |
| 8 | `5V`/main supply | Teensy `3.3V` output, tied to P1-7 |

This uses the AS5048A's documented 3.3 V supply mode. The breakout's `3.3V` and `5V`/main-supply leads must be tied together and connected to the Teensy's `3.3V` output. If those two breakout leads are already combined, connect that single combined lead to Teensy `3.3V`.

**Never connect the combined supply lead to Teensy `VIN`, `VUSB`, or any 5 V source.** The Teensy 4.1 `3.3V` pin is an output in this setup; power the Teensy normally through its USB device connector. The AS5048A draws far less than the Teensy's stated 250 mA budget for external 3.3 V circuitry.

Keep the SPI wires short and away from motor phase or power cables. This program uses SPI mode 1 at 1 MHz.

## Magnet

Use a diametrically magnetized two-pole magnet, centred over or under the AS5048A. The adapter-board manual specifies a 0.5–2 mm air gap, centring within 0.5 mm, and a non-ferromagnetic holder.

## Build, test, and upload

Install PlatformIO, open a terminal in this directory, and run:

```sh
pio test -e native
pio run -e teensy41
pio run -e teensy41 -t upload
pio device monitor -b 115200
```

If upload waits for the board, briefly press the Teensy's Program button.

## Local web dashboard

After uploading the latest firmware, start the included dependency-free local server:

```sh
python3 web/serve.py
```

Open [http://localhost:8765](http://localhost:8765) in Chrome or Edge, then:

1. Click **Connect Teensy**.
2. Choose the Teensy USB serial device in the browser prompt.
3. Confirm that the angle, counts, health, chart, and terminal stream begin updating.
4. Move the pendulum or magnet to the desired reference position.
5. Click **Zero current position**. The firmware stores that absolute count as the zero offset until it restarts.

The page automatically reconnects on later visits once the browser has permission. Browser security requires the first serial-port selection to come from your click. Add `?demo=1` to the URL to preview and test the interface without connected hardware.

The dashboard runs only on `127.0.0.1`; it is not exposed to other machines on the network. Close the server terminal or press `Ctrl+C` to stop it.

## Terminal commands

You can also type these commands into any 115200-baud serial terminal:

- `zero` — make the encoder's current position 0 degrees.
- `help` — print the supported command list.

## Expected output

The monitor prints 100 samples per second:

```text
time_us,absolute_count,zeroed_count,angle_deg,agc,status,parity_errors,protocol_errors
3021981,5319,5319,116.8726,121,OK,0,0
# ZEROED,5319
```

Rotate the magnet slowly by hand:

1. `absolute_count` should move smoothly through `0`–`16383` and wrap once per revolution.
2. `zeroed_count` should read approximately `0` immediately after the `zero` command.
3. `angle_deg` should move through `0`–just under `360` degrees.
4. One quarter-turn should change the count by approximately 4096.
5. One complete turn should return close to the original count.
6. `status` should remain `OK` and both error counters should remain zero.

## Troubleshooting

- `MAGNET_TOO_WEAK`: reduce the air gap, improve centring, or verify the magnet is diametric.
- `MAGNET_TOO_STRONG`: increase the air gap.
- `CORDIC_OVERFLOW`: check magnet type, centring, and distance.
- `OCF_NOT_READY` continuously: check sensor power and magnet placement.
- Increasing `parity_errors`: shorten/separate SPI wiring and try lowering `kSpiClockHz` in `src/main.cpp` to `100000`.
- Increasing `protocol_errors`: check CS/SCK/MOSI wiring and ensure no other device drives the SPI bus.
- Constant or missing output: verify breakout power, shared ground, MISO/MOSI orientation, and the selected serial port.
