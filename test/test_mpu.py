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

@cocotb.test()
async def test_mpu_init_and_read(dut):
    """Test MPU initialization and data reading sequence."""

    # Start clock
    cocotb.start_soon(generate_clock(dut))

    # Set initial inputs
    dut.spi_miso.value = 0

    # Reset
    await reset(dut)

    dut._log.info("Waiting for Initialization...")

    # Wait for S_WAKE_1 (state 1)
    # With INIT_WAIT_CYCLES=10, this should be fast.
    while dut.state.value != 1:
        await RisingEdge(dut.clk)

    dut._log.info("Entered S_WAKE_1 (Write PWR_MGMT_1)")

    # Wait for next cycle to see effects of S_WAKE_1 (CS_N going low, State going to WAIT)
    await RisingEdge(dut.clk)

    assert dut.state.value == 2 # S_WAKE_1_WAIT
    assert dut.spi_cs_n.value == 0

    # Wait for SPI Start pulse to propagate to shift_reg
    await RisingEdge(dut.clk)
    # Check if correct byte is being sent (0x6B)
    # Accessing internal signal spi_inst.shift_reg might depend on hierarchy
    # assert dut.spi_inst.shift_reg.value == 0x6B

    # Wait for transaction to complete (S_WAKE_1_WAIT -> S_WAKE_2)
    while dut.state.value != 3:
        await RisingEdge(dut.clk)

    dut._log.info("Entered S_WAKE_2 (Write 0x00)")
    assert dut.spi_cs_n.value == 0

    # Wait for transaction to complete (S_WAKE_2_WAIT -> S_IDLE)
    while dut.state.value != 5:
        await RisingEdge(dut.clk)

    dut._log.info("Entered S_IDLE")
    assert dut.spi_cs_n.value == 1

    # Wait for S_READ_CMD (state 6)
    # TIMER_LIMIT=20, should be fast.
    while dut.state.value != 6:
        await RisingEdge(dut.clk)

    dut._log.info("Entered S_READ_CMD (Read ACCEL_XOUT_H)")

    # Wait for next cycle
    await RisingEdge(dut.clk)

    assert dut.state.value == 7 # S_READ_CMD_WAIT
    assert dut.spi_cs_n.value == 0

    # Wait for S_READ_BYTES (state 8)
    while dut.state.value != 8:
        await RisingEdge(dut.clk)

    dut._log.info("Entered S_READ_BYTES loop")

    # We expect 14 bytes to be read.
    # We can mock MISO data.
    # For simplicity, let's just toggle MISO to produce some pattern.

    for i in range(14):
        # Wait for start of byte transaction
        while dut.spi_start.value == 0:
            await RisingEdge(dut.clk)

        # During the SPI transaction, set MISO
        # spi_master runs for roughly 16 * CLK_DIV cycles.
        # We can just wait for done.

        # Set a dummy value on MISO (e.g., 0xAB for all bytes)
        # MISO is sampled on Rising Edge of SCLK.
        # To make it simple, we just set MISO = 1 constant, so we read 0xFF.
        dut.spi_miso.value = 1

        while dut.spi_done.value == 0:
            await RisingEdge(dut.clk)

        dut._log.info(f"Read byte {i}")

    # Wait for valid signal
    while dut.valid.value == 0:
        await RisingEdge(dut.clk)

    dut._log.info("Valid signal received")

    # Check values
    # If MISO was 1, we read 0xFF for all bytes.
    # Accel X = 0xFFFF (-1)
    assert dut.accel_x.value == 0xFFFF
    assert dut.accel_y.value == 0xFFFF
    assert dut.accel_z.value == 0xFFFF

    dut._log.info("Test passed!")
