`ifndef PARAMS_VH
`define PARAMS_VH

// System Clock Frequency in Hz (Default 10 MHz for TinyTapeout)
parameter SYS_CLK_FREQ = 10000000;

// SPI Clock Frequency in Hz (Default 1 MHz)
parameter SPI_CLK_FREQ = 1000000;

// UART Baud Rate (Default 9600)
parameter UART_BAUD_RATE = 9600;

// SPI Clock Divider (SYS_CLK / (2 * SPI_CLK)) - approximate
parameter SPI_CLK_DIV = SYS_CLK_FREQ / (2 * SPI_CLK_FREQ);

// UART Baud Divider (SYS_CLK / BAUD_RATE)
parameter UART_BAUD_DIV = SYS_CLK_FREQ / UART_BAUD_RATE;

// MPU-6500 Register Addresses
parameter MPU_PWR_MGMT_1 = 8'h6B;
parameter MPU_ACCEL_XOUT_H = 8'h3B;
parameter MPU_GYRO_CONFIG = 8'h1B;
parameter MPU_ACCEL_CONFIG = 8'h1C;

`endif
