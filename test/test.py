import cocotb
from cocotb.triggers import RisingEdge, Timer

async def reset(dut):
    dut.rst_n.value = 0
    dut.clk.value = 0
    await Timer(100, units="ns")
    dut.rst_n.value = 1
    await Timer(100, units="ns")

async def generate_clock(dut):
    while True:
        dut.clk.value = 0
        await Timer(50, units="ns") # 10MHz
        dut.clk.value = 1
        await Timer(50, units="ns")

async def spi_miso_driver(dut):
    while True:
        await RisingEdge(dut.clk)
        try:
            # Read full vector
            val = dut.uo_out.value
            # Check bit 2 (CS_N)
            val_int = int(val)
            cs_val = (val_int >> 2) & 1
        except:
            cs_val = 1

        if cs_val == 0:
            dut.ui_in.value = 1 # Toggle bit 0 (MISO)
        else:
            dut.ui_in.value = 0

@cocotb.test()
async def test_top_level(dut):
    cocotb.start_soon(generate_clock(dut))
    cocotb.start_soon(spi_miso_driver(dut))

    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.ena.value = 1

    dut._log.info("Applying Reset...")
    await reset(dut)
    dut._log.info("Reset Released.")

    # Wait for UART
    timeout = 10000
    detected = False
    prev_val = 1

    for i in range(timeout):
        await RisingEdge(dut.clk)
        try:
            val_int = int(dut.uo_out.value)
            uart_val = (val_int >> 3) & 1 # Bit 3
        except:
            uart_val = 1

        if prev_val == 1 and uart_val == 0:
            dut._log.info(f"UART Start Bit Detected at cycle {i}!")
            detected = True
            break
        prev_val = uart_val

    if not detected:
        dut._log.error("Timeout waiting for UART.")
        assert False, "Timeout"

    # Proceed with decoding (Fast)
    bit_period = 5
    for _ in range(bit_period // 2): await RisingEdge(dut.clk)
    byte_val = 0
    for bit_idx in range(8):
        for _ in range(bit_period): await RisingEdge(dut.clk)
        try:
            val_int = int(dut.uo_out.value)
            bit = (val_int >> 3) & 1
        except: bit = 0
        if bit == 1: byte_val |= (1 << bit_idx)
    dut._log.info(f"Received: {hex(byte_val)}")

    if byte_val != 0xDE:
        dut._log.error(f"Header Mismatch: Expected 0xDE, got {hex(byte_val)}")
        assert False, "Header Mismatch"
    else:
        dut._log.info("Header 0xDE Verified! GLS Test Passed.")
