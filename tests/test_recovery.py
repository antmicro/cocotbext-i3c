#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

import cocotb
from cocotb.triggers import Timer
from utils import I3cTestbench

from cocotbext_i3c.i3c_recovery_interface import I3cRecoveryInterface


@cocotb.test(skip=True)
async def test_recovery(dut):
    """
    FIXME: Test fails due to incorrect handling of repeated start in transfer:
    'I3C Private Read Transfer (from Repeated Start) after Private Write'.
    The issue lies in the implementation of recv_byte method of the i3c_target model.

    `command_read` awaits for the `_i3c_recovery_read`, which awaits the private read:
    `await self.controller.send_start()`. At this point, the i3c_target model already
    made a decision that the incoming transfer is a continuation of the write data transfer.
    Instead, the model should keep track of bus state, because a repeated start could arrive.
    """
    address = 0x5A
    tb = I3cTestbench(dut, tgt_address=address)
    rec_if = I3cRecoveryInterface(tb.i3c_controller)

    command = I3cRecoveryInterface.Command.PROT_CAP
    data = [0x24, 0x25, 0x26]

    force_pec_error = True
    for force_pec_error in [False, True]:
        await rec_if.command_write(address, command, data, force_pec_error)
        await rec_if.command_read(address, command, force_pec_error)

    await Timer(10, "ns")
