# Furuta pendulum controllers

This repository now keeps the two hardware implementations separate:

- [`legacy_teensy_can/`](legacy_teensy_can/) is the original Teensy 4.1 + CAN firmware.
- [`raspberry_pi_usb/`](raspberry_pi_usb/) is a temporary Raspberry Pi 5 controller that talks directly to the ODrive S1 over isolated USB and reads the pendulum encoder plus safety loop from Pi GPIO.

Start with the README inside the implementation you intend to use. Both versions start disarmed and require an explicit `zero` followed by `arm`.

> **Experimental machinery warning:** A Furuta pendulum can move suddenly and cause injury. Use guards, physical travel stops, conservative current/torque limits, and a hard-wired E-stop that removes motor power. Neither a Raspberry Pi GPIO input nor an ODrive enable input is a safety-rated E-stop.

The Pi/USB version is useful for bring-up while a CAN interface is unavailable, but it is not a like-for-like microcontroller replacement. Raspberry Pi OS is not a hard real-time operating system, GPIO encoder events can be lost under load, and ODrive recommends CAN rather than USB for runtime operation. Validate loop timing and encoder counts with motor power disconnected before attempting motion.
