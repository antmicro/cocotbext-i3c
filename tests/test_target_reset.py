import cocotb
from cocotb.triggers import Timer
from utils import I3cTestbench

from cocotbext_i3c.common import I3cTargetResetAction


@cocotb.test()
async def test_simple_broadcast_default_target_reset(dut):
    address = 0x40
    tb = I3cTestbench(dut, address)

    # Should send a target reset pattern with no configuration
    await tb.i3c_controller.target_reset()

    await Timer(500, "ns")


@cocotb.test()
async def test_simple_broadcast_target_reset(dut):
    address = 0x40
    tb = I3cTestbench(dut, address)

    # Should broadcast configuration and send trarget reset pattern
    await tb.i3c_controller.target_reset(reset_actions=I3cTargetResetAction.RESET_PERIPHERAL_ONLY)

    await Timer(500, "ns")


@cocotb.test()
async def test_multiple_direct_target_reset(dut):
    address = 0x40
    tb = I3cTestbench(dut, address)

    # Should produce two chained direct CCCs to configure three targets and then send
    # target reset pattern
    await tb.i3c_controller.target_reset(
        reset_actions=[
            (0x20, I3cTargetResetAction.RESET_PERIPHERAL_ONLY),
            (0x21, I3cTargetResetAction.RESET_WHOLE_TARGET),
            (0x22, I3cTargetResetAction.RESET_WHOLE_TARGET),
        ]
    )

    await Timer(500, "ns")
