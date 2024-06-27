#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

import logging
import os


import cocotb
from cocotb.triggers import Timer

from cocotbext_i3c.i3c_controller import I3cController, I3cXferMode
from cocotb.types import Logic


class TB:
    def __init__(self, dut):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        self.i3c_controller = I3cController(
            sda_i=dut.sda_o,
            sda_o=dut.sda_i,
            scl_i=dut.scl_o,
            scl_o=dut.scl_i,
            debug_state_o=dut.debug_state_i,
            speed=12.5e6,
            silent=False
        )


@cocotb.test()
async def test_i3c(dut):

    dut.pull_sda_low_i.value = 0;

    tb = TB(dut)

    await Timer(100, "ns")

    # Private write
    await tb.i3c_controller.i3c_write(0x50, [0xaa])

    await Timer(300, "ns")

    # Legacy I2C Write
    await tb.i3c_controller.i3c_write(0x50, b"\xbb", mode=I3cXferMode.LEGACY_I2C)

    await Timer(300, "ns")

    # Private read

    # Should read two byte as the target keeps SDA high
    dut.pull_sda_low_i.value = 0
    data = await tb.i3c_controller.i3c_read(0x50, 2)
    assert len(data) == 2

    # Should read one byte as the target keeps SDA low
    dut.pull_sda_low_i.value = 1
    data = await tb.i3c_controller.i3c_read(0x50, 2)
    assert len(data) == 1

    await Timer(300, "ns")

    # Legacy I2C Read
    data = await tb.i3c_controller.i3c_read(0x50, 2, mode=I3cXferMode.LEGACY_I2C)

    await Timer(300, "ns")

    # Broadcasted SETMRL CCC
    await tb.i3c_controller.i3c_ccc_write(0xA, broadcast_data=[0xaa, 0xbb])

    await Timer(300, "ns")

    # Directed SETMRL CCC @ 0x50 and 0x51
    await tb.i3c_controller.i3c_ccc_write(0x8A, directed_data=[
        (0x50, [0xaa, 0xbb]),
        (0x51, [0xff, 0x00])
    ])

    await Timer(300, "ns")

    dut.pull_sda_low_i.value = 0
    mrl_data = await tb.i3c_controller.i3c_ccc_read(0x8C, 0x51, 2)
    assert len(mrl_data) == 2
