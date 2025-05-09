#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

from random import randint

import cocotb
from cocotb.triggers import Timer
from utils import I3cExecError, I3cTestbench


async def test_simple_read(dut, write_data):
    """
    Writes data directly to the I3C Target memory model and issues a private
    read to the I3C Target.
    Verifies the read data.
    """
    tb = I3cTestbench(dut)

    await Timer(100, "ns")

    length = len(write_data)
    tb.i3c_target._mem._write_mem(0, write_data, length)
    resp = await tb.i3c_controller.i3c_read(tb.i3c_target.address, length)
    assert not resp.nack

    # Dump memory for more insight
    if resp.data != bytearray(write_data):
        tb.log.info("Dump target memory")
        tb.i3c_target._mem.dump()
        raise I3cExecError(
            f"Written {[hex(_) for _ in write_data]} to the target device but read {resp.data}"
        )


@cocotb.test()
async def test_simple_read_odd_byte(dut):
    await test_simple_read(dut, [0xA0])


@cocotb.test()
async def test_simple_read_even_byte(dut):
    await test_simple_read(dut, [0xAA])


@cocotb.test()
async def test_simple_read_zero(dut):
    await test_simple_read(dut, [0x00 for _ in range(42)])


@cocotb.test()
async def test_simple_read_all_bits(dut):
    await test_simple_read(dut, [0xFF for _ in range(42)])


@cocotb.test()
async def test_simple_read_long(dut):
    await test_simple_read(
        dut,
        [0x00, 0xBB, 0x00, 0x0C, 0xC0, 0x00, 0x10, 0x01, 0x00, 0x2A, 0xE3, 0xAF, 0xAC, 0xDC, 0x04],
    )


@cocotb.test()
async def test_simple_write_randomized(dut):
    data_len = 100
    data = [randint(0, 255) for _ in range(data_len)]
    await test_simple_read(dut, data)
