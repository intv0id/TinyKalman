# Kalman Filter on TinyTapeout

![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

This project implements a simplified Kalman Filter for MPU-6500 sensor fusion on TinyTapeout ASIC and FPGA.

## Features

*   **Sensor Interface**: SPI Master for MPU-6500 (Accelerometer + Gyroscope).
*   **Angle Calculation**: CORDIC-based `atan2` calculation for Roll and Pitch from accelerometer data.
*   **Sensor Fusion**: Steady-state Kalman Filter (Complementary Filter) to fuse Gyroscope rate with Accelerometer angle.
*   **Output**: UART Serial output (9600 baud) streaming Roll, Pitch, and Yaw.

## Algorithm Details

### 1. Angle Estimation (CORDIC)
Roll ($\phi$) and Pitch ($\theta$) angles are calculated from the accelerometer vector using an iterative **CORDIC** (COordinate Rotation DIgital Computer) algorithm in vectoring mode. This calculates `atan2(y, x)` efficiently without multipliers.

*   **Roll**: $\phi = \text{atan2}(a_y, a_z)$
*   **Pitch**: $\theta = \text{atan2}(-a_x, \sqrt{a_y^2 + a_z^2})$

*Note: In this implementation, the Pitch calculation uses a simplified approximation where the magnitude output from the Roll CORDIC is used as the denominator.*

### 2. Sensor Fusion (Complementary / Kalman Filter)
The design uses a steady-state 1D Kalman Filter (mathematically equivalent to a Complementary Filter) for both Roll and Pitch axes. This fuses the noisy but stable accelerometer angle with the precise but drifting gyroscope rate.

**Equations:**
1.  **Prediction (Gyro Integration):**
    $$ \theta_{pred}[k] = \theta_{est}[k-1] + (\text{GyroRate} \times \Delta t) $$
    *Implemented as:* `pred_angle = angle_out + (rate >>> 6)`

2.  **Update (Accelerometer Correction):**
    $$ \theta_{est}[k] = \theta_{pred}[k] + K \times (\theta_{acc}[k] - \theta_{pred}[k]) $$
    *Implemented as:* `angle_out = pred_angle + ((angle_m - pred_angle) >>> 6)`

*   **Gain ($K$):** The Kalman Gain is fixed at $1/64$ (`>>> 6`), balancing responsiveness and noise rejection for a standard 100Hz IMU loop.
*   **Time Step ($\Delta t$):** The rate shift factor (`>>> 6`) implicitly handles the scaling for the time step and gyro sensitivity.

### 3. Yaw Integration
Yaw ($\psi$) is calculated by simple integration of the Z-axis gyroscope rate, as there is no magnetometer for correction.
*   $\psi[k] = \psi[k-1] + (\text{GyroRate}_z \times \Delta t)$

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
