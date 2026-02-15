# Kalman Filter on TinyTapeout

![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

This project implements a simplified Kalman Filter for MPU-6500 sensor fusion on TinyTapeout ASIC and FPGA.

## Features

*   **Sensor Interface**: SPI Master for MPU-6500 (Accelerometer + Gyroscope).
*   **Angle Calculation**: CORDIC-based `atan2` calculation for Roll and Pitch from accelerometer data.
*   **Sensor Fusion**: Steady-state Kalman Filter (Complementary Filter) to fuse Gyroscope rate with Accelerometer angle.
*   **Output**: UART Serial output (9600 baud) streaming Roll, Pitch, and Yaw.

## Pinout

| Pin | Function | Description |
|---|---|---|
| `ui_in[0]` | **MISO** | SPI Master In Slave Out (from Sensor) |
| `uo_out[0]` | **MOSI** | SPI Master Out Slave In (to Sensor) |
| `uo_out[1]` | **SCLK** | SPI Clock |
| `uo_out[2]` | **CS_N** | SPI Chip Select (Active Low) |
| `uo_out[3]` | **TX** | UART Transmit (to PC) |
| `clk` | **CLK** | System Clock (10MHz default) |
| `rst_n` | **RST** | Reset (Active Low) |

## Data Format

The device outputs a continuous stream of 8-byte packets at 9600 baud.

| Byte | Value |
|---|---|
| 0 | `0xDE` (Header) |
| 1 | `0xAD` (Header) |
| 2 | Roll (High Byte) |
| 3 | Roll (Low Byte) |
| 4 | Pitch (High Byte) |
| 5 | Pitch (Low Byte) |
| 6 | Yaw (High Byte) |
| 7 | Yaw (Low Byte) |

Angles are 16-bit signed integers. Scale: `32768 = 180 degrees`.

## FPGA Implementation

The design is compatible with iCE40 FPGAs (specifically tested for iCEBreaker).
See `fpga/` directory for build files.

To build for iCEBreaker:
```bash
cd fpga
make prog
```
Note: The FPGA build uses a 12MHz clock configuration.

## Simulation

To run the testbench (using Cocotb):
```bash
cd test
make
```

## License

Apache 2.0
