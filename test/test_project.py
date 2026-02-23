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
    # Wait for CS low
    while True:
        if dut.spi_cs_n.value == 0:
            # Transfer starts
            # Read command (first byte)
            cmd = 0
            for i in range(8):
                await RisingEdge(dut.spi_sclk)
                bit = int(dut.spi_mosi.value)
                cmd = (cmd << 1) | bit

            # If Read command (0x3B | 0x80 = 0xBB)
            if cmd == 0xBB:
                # Send 14 bytes
                # Mock Data:
                # Accel X = 0 (0x0000)
                # Accel Y = 0 (0x0000)
                # Accel Z = 16384 (1G) (0x4000) -> Roll 0
                # Temp
                # Gyro X = 0
                # Gyro Y = 0
                # Gyro Z = 640 (Rate test) (0x0280) -> Yaw increase

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
                    # MISO is driven on falling edge (MPU) for master to sample on rising
                    # Master samples on rising.
                    # We should drive on falling edge of SCLK?
                    # Or just drive before rising edge.
                    # SPI Mode 3: Master drives MOSI on Falling, Samples MISO on Rising.
                    # Slave samples MOSI on Rising, Drives MISO on Falling.

                    for i in range(8):
                        # Wait for falling edge to drive MISO
                        while dut.spi_sclk.value == 1:
                            await RisingEdge(dut.clk)
                            if dut.spi_cs_n.value == 1: return

                        # SCLK is now 0 (or going to 0)
                        bit = (byte >> (7-i)) & 1
                        dut.spi_miso.value = bit

                        # Wait for rising edge
                        while dut.spi_sclk.value == 0:
                            await RisingEdge(dut.clk)
                            if dut.spi_cs_n.value == 1: return

            # If Write command (0x6B = 0x6B)
            # Just ignore data
            pass

        await RisingEdge(dut.clk)

async def read_uart_byte(dut, baud_rate=9600):
    """Decodes a single byte from the UART TX line."""
    # Calculate bit time in ns (1e9 / 9600 ~= 104166.67 ns)
    bit_time_ns = 1_000_000_000 / baud_rate

    # Wait for Start Bit (Falling Edge)
    while dut.uart_tx_out.value == 1:
        await RisingEdge(dut.clk)

    # Wait 1.5 bit times to sample the middle of bit 0
    await Timer(bit_time_ns * 1.5, units="ns")

    byte = 0
    for i in range(8):
        if dut.uart_tx_out.value == 1:
            byte |= (1 << i)
        await Timer(bit_time_ns, units="ns")

    return byte

@cocotb.test()
async def test_top_level(dut):
    """Test full system integration."""
    cocotb.start_soon(generate_clock(dut))
    cocotb.start_soon(spi_slave_mock(dut))

    await reset(dut)

    dut._log.info("Waiting for UART output...")

    # Read 8 bytes from the UART stream
    received_bytes = []
    for i in range(8):
        byte = await read_uart_byte(dut)
        received_bytes.append(byte)
        dut._log.info(f"Received UART Byte {i}: {hex(byte)}")

    # 1. Verify Header
    assert received_bytes[0] == 0xDE
    assert received_bytes[1] == 0xAD
    dut._log.info("Header verified!")

    # 2. Verify Yaw (Bytes 6 and 7)
    # Gyro Z input was 640.
    # Expected calculation: Yaw += (Rate >>> 6)
    # 640 / 64 = 10.
    yaw_h = received_bytes[6]
    yaw_l = received_bytes[7]
    yaw_val = (yaw_h << 8) | yaw_l
    dut._log.info(f"Yaw Value: {yaw_val}")

    assert yaw_val == 10
    dut._log.info("Test Passed!")
