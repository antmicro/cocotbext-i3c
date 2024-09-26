from random import randint

import cocotb
from cocotb.triggers import Timer
from utils import I3cExecError, I3cTestbench
from cocotbext_i3c.common import I3cTargetResetAction

@cocotb.test()
async def test_simple_target_reset(dut):
    address = 0x40
    tb = I3cTestbench(dut, address)

    await tb.i3c_controller.target_reset(
        reset_actions=I3cTargetResetAction.RESET_PERIPHERAL_ONLY
    )

@cocotb.test()
async def test_multiple_target_reset(dut):
    address = 0x40
    tb = I3cTestbench(dut, address)

    await tb.i3c_controller.target_reset(
        reset_actions=[
            (0x20, I3cTargetResetAction.RESET_PERIPHERAL_ONLY),
            (0x21, I3cTargetResetAction.RESET_WHOLE_TARGET)
        ]
    )