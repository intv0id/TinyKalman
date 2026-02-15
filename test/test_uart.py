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
async def test_uart_tx(dut):
    """Test UART TX sending a byte."""
    cocotb.start_soon(generate_clock(dut))
    await reset(dut)

    # Send 'A' (0x41) = 0100 0001
    # LSB First: 1 0 0 0 0 0 1 0
    # Expected sequence: Start(0), 1, 0, 0, 0, 0, 0, 1, 0, Stop(1)

    dut.data_in.value = 0x41
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    # Wait for busy
    await RisingEdge(dut.clk)
    assert dut.busy.value == 1

    # Wait for completion
    while dut.done.value == 0:
        await RisingEdge(dut.clk)

    assert dut.busy.value == 0
    assert dut.tx.value == 1 # Idle high

    dut._log.info("UART TX Test Passed!")
