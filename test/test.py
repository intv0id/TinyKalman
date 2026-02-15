import cocotb
from cocotb.triggers import RisingEdge, Timer

async def reset(dut):
    dut.rst_n.value = 0
    dut.clk.value = 0
    await Timer(10, units="ns")
    dut.rst_n.value = 1
    await Timer(10, units="ns")

async def generate_clock(dut):
    while True:
        dut.clk.value = 0
        await Timer(50, units="ns") # 10MHz
        dut.clk.value = 1
        await Timer(50, units="ns")

async def spi_slave_mock(dut):
    """Mocks the MPU-6500 SPI Slave."""
    # Access signals from user_project
    # dut is tb. dut.user_project is tt_um_kalman

    # SPI Pins are on uo_out (outputs from FPGA, inputs to Slave)
    # MOSI = uo_out[0], SCLK = uo_out[1], CS_N = uo_out[2]
    # MISO = ui_in[0] (Input to FPGA)

    # However, we can also access internal signals of user_project for easier naming
    # internal: spi_mosi, spi_sclk, spi_cs_n

    # Wait for CS low
    while True:
        if dut.user_project.spi_cs_n.value == 0:
            # Transfer starts
            # Read command (first byte)
            cmd = 0
            for i in range(8):
                await RisingEdge(dut.user_project.spi_sclk)
                bit = int(dut.user_project.spi_mosi.value)
                cmd = (cmd << 1) | bit

            # If Read command (0x3B | 0x80 = 0xBB)
            if cmd == 0xBB:
                # Send 14 bytes
                # Mock Data: Z=1G (0x4000), GyroZ=640 (0x0280)
                data = [
                    0x00, 0x00, # Accel X
                    0x00, 0x00, # Accel Y
                    0x40, 0x00, # Accel Z
                    0x00, 0x00, # Temp
                    0x00, 0x00, # Gyro X
                    0x00, 0x00, # Gyro Y
                    0x02, 0x80  # Gyro Z
                ]

                for byte in data:
                    for i in range(8):
                        # Wait for falling edge to drive MISO
                        while dut.user_project.spi_sclk.value == 1:
                            await RisingEdge(dut.clk)
                            if dut.user_project.spi_cs_n.value == 1: return

                        # SCLK is now 0
                        bit = (byte >> (7-i)) & 1
                        # MISO is ui_in[0]
                        dut.ui_in.value = bit

                        # Wait for rising edge
                        while dut.user_project.spi_sclk.value == 0:
                            await RisingEdge(dut.clk)
                            if dut.user_project.spi_cs_n.value == 1: return

            pass

        await RisingEdge(dut.clk)

@cocotb.test()
async def test_top_level(dut):
    """Test full system integration."""
    cocotb.start_soon(generate_clock(dut))
    cocotb.start_soon(spi_slave_mock(dut))

    # Initialize inputs
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.ena.value = 1

    await reset(dut)

    dut._log.info("Waiting for UART output...")

    # Access UART internal signals
    uart_tx = dut.user_project.uart_tx_out

    # Wait for UART Start bit (TX goes low)
    while uart_tx.value == 1:
        await RisingEdge(dut.clk)

    dut._log.info("UART Start Bit Detected!")

    # Verify we get the sequence 0xDE, 0xAD...
    # Snoop internal signals
    while dut.user_project.state.value != 6: # S_SEND_UART
        await RisingEdge(dut.clk)

    expected_data = [0xDE, 0xAD]

    for expected in expected_data:
        while dut.user_project.uart_data.value != expected:
            await RisingEdge(dut.clk)

        dut._log.info(f"Seen UART Data: {hex(expected)}")

        while dut.user_project.uart_done.value == 0:
            await RisingEdge(dut.clk)
        while dut.user_project.uart_done.value == 1:
             await RisingEdge(dut.clk)

    dut._log.info("Header verified!")

    # Check Yaw
    for i in range(4):
        while dut.user_project.uart_done.value == 0:
            await RisingEdge(dut.clk)
        while dut.user_project.uart_done.value == 1:
             await RisingEdge(dut.clk)

    while dut.user_project.uart_cnt.value != 6:
        await RisingEdge(dut.clk)

    yaw_h = int(dut.user_project.uart_data.value)

    while dut.user_project.uart_done.value == 0:
        await RisingEdge(dut.clk)
    while dut.user_project.uart_done.value == 1:
         await RisingEdge(dut.clk)

    while dut.user_project.uart_cnt.value != 7:
        await RisingEdge(dut.clk)

    yaw_l = int(dut.user_project.uart_data.value)

    yaw_val = (yaw_h << 8) | yaw_l
    dut._log.info(f"Yaw Value: {yaw_val}")

    assert yaw_val == 10 or yaw_val == 0

    dut._log.info("Test Passed!")
