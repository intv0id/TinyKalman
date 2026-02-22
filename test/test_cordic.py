import cocotb
from cocotb.triggers import RisingEdge, Timer
import math

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
async def test_cordic_atan2(dut):
    """Test CORDIC atan2 function."""
    cocotb.start_soon(generate_clock(dut))
    await reset(dut)

    # Test cases: (x, y) -> expected angle
    # Scale: 32768 = 180 deg.
    test_cases = [
        (10000, 10000, 45.0),      # 45 deg
        (10000, 0, 0.0),           # 0 deg
        (0, 10000, 90.0),          # 90 deg
        (-10000, 10000, 135.0),    # 135 deg
        (-10000, -10000, -135.0),  # -135 deg
        (10000, -10000, -45.0),    # -45 deg
        # Edge cases
        (0, 0, 0.0),               # (0,0) -> 0 deg (Undefined but expected to be stable)
        (32767, 0, 0.0),           # Max positive X
        (0, 32767, 90.0),          # Max positive Y
        (-32768, 0, 180.0),        # Min negative X
        (0, -32768, -90.0),        # Min negative Y
        (32767, 32767, 45.0),      # Max positive X, Max positive Y
        (-32768, -32768, -135.0),  # Min negative X, Min negative Y
        (-32768, 1, 180.0),        # Quadrant boundary (upper)
        (-32768, -1, -180.0),      # Quadrant boundary (lower)
    ]

    for x, y, expected_deg in test_cases:
        dut.x_in.value = x
        dut.y_in.value = y
        dut.start.value = 1
        await RisingEdge(dut.clk)
        dut.start.value = 0

        while dut.done.value == 0:
            await RisingEdge(dut.clk)

        angle_raw = dut.angle_out.value.signed_integer
        # Convert raw to degrees: raw * 180 / 32768
        angle_deg = angle_raw * 180.0 / 32768.0

        dut._log.info(f"Input: ({x}, {y}), Output: {angle_deg:.2f} deg, Expected: {expected_deg} deg")

        # 1 degree tolerance is generous but reasonable for 16-bit fixed point CORDIC with 16 stages
        assert abs(angle_deg - expected_deg) < 1.0
