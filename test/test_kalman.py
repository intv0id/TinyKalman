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
async def test_kalman_filter(dut):
    """Test Kalman Filter convergence."""
    cocotb.start_soon(generate_clock(dut))
    await reset(dut)

    # Initial state
    dut.en.value = 0
    dut.rate.value = 0
    dut.angle_m.value = 0
    await RisingEdge(dut.clk)

    # Test 1: Static Angle. Rate = 0. Angle_m = 1000.
    # Should converge to 1000.
    target_angle = 1000
    dut.angle_m.value = target_angle
    dut.rate.value = 0

    dut._log.info("Starting convergence test...")
    for i in range(300):
        dut.en.value = 1
        await RisingEdge(dut.clk)
        dut.en.value = 0 # Pulse enable
        # Wait a bit
        await RisingEdge(dut.clk)

        curr_angle = dut.angle_out.value.signed_integer
        # dut._log.info(f"Iter {i}: Angle {curr_angle}")

    final_angle = dut.angle_out.value.signed_integer
    dut._log.info(f"Final Angle: {final_angle}, Target: {target_angle}")
    assert abs(final_angle - target_angle) < 10 # Tolerance

    # Test 2: Rate Integration.
    # Angle_m = 0 (ignored or constant error).
    # Rate = 100.
    # Angle should increase by Rate >> RATE_SHIFT per step.
    # RATE_SHIFT = 6 (default). 100 >> 6 = 1.
    # K_SHIFT = 6 (default).

    # Reset filter
    await reset(dut)
    dut.angle_m.value = 0
    dut.rate.value = 640 # 640 >> 6 = 10 per step.

    dut._log.info("Starting rate integration test...")
    for i in range(10):
        dut.en.value = 1
        await RisingEdge(dut.clk)
        dut.en.value = 0
        await RisingEdge(dut.clk)

    # Expected: 10 steps * 10 = 100.
    # But measured is 0. So update term pulls it back.
    # Pred = Angle + 10.
    # Angle = Pred + K * (0 - Pred) = Pred - K * Pred = Pred * (1 - K).
    # So it will decay if measured is 0.

    # To test rate only, we set K very small? No, K is fixed.
    # Just verify behavior is consistent.
    # Angle[n] = (Angle[n-1] + 10) * (63/64).
    # Approx 10 + 10 + ... but slightly less.

    final_angle = dut.angle_out.value.signed_integer
    dut._log.info(f"Final Angle with Rate: {final_angle}")
    assert final_angle > 0
