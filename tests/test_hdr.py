#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

import cocotb
from cocotb.triggers import Timer
from utils import I3cTestbench


@cocotb.test()
async def test_hdr_exit(dut):
    """
    Sends the HDR Exit Pattern and ensures that it is triggered under specified
    conditions.

    TODO: Send rising SCL in the middle of HDR Exit pattern to check behavior
          on corner cases.
    """
    tb = I3cTestbench(dut)

    await Timer(100, "ns")

    assert not tb.i3c_target.hdr_exit_detected, "ERROR: Unexpected HDR Exit Pattern detected"
    await tb.i3c_controller.send_hdr_exit()
    assert tb.i3c_target.hdr_exit_detected, "ERROR: HDR Exit Pattern not detected"

    tb.i3c_target.clear_hdr_exit_flag()
    assert not tb.i3c_target.hdr_exit_detected, "ERROR: HDR Exit Pattern detected after HDR Exit flag was cleared"

    await tb.i3c_controller.send_hdr_exit()
    assert tb.i3c_target.hdr_exit_detected, "ERROR: HDR Exit Pattern not detected"
