# Teensy 4.1 to ODrive S1 UART probe

This temporary firmware diagnoses the isolated UART link without running any
motor-control code. It cycles through plain and checksummed read-only bus
voltage, active-error, and position/velocity queries, and prints the raw ODrive
responses over the Teensy's USB serial connection.

Wiring is the same as the main controller:

| Teensy 4.1 | ODrive S1 isolated I/O |
| --- | --- |
| pin 0 / RX1 | G06 / UART TX |
| pin 1 / TX1 | G07 / UART RX |
| 3.3 V | V+ ISO |
| GND | GND ISO |

Build, upload, and monitor from this directory:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

A working link prints a numeric voltage near the powered DC-bus voltage. A
repeated `<no bytes received>` means the Teensy is not seeing any UART response.
Re-upload the firmware in `teensy_uart_web` after diagnosis to restore the full
controller.
