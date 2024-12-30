#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

from random import choice, randint

import cocotb
from cocotb.triggers import Timer
from utils import I3cTestbench

from cocotbext_i3c.i3c_controller import I3cXferMode


@cocotb.test()
async def test_i3c(dut):
    tb = I3cTestbench(dut)

    await Timer(100, "ns")

    # Private write
    await tb.i3c_controller.i3c_write(0x50, [0xAA, 0xBB])

    await Timer(300, "ns")

    # Legacy I2C Write
    await tb.i3c_controller.i3c_write(0x42, b"\xbb", mode=I3cXferMode.LEGACY_I2C)

    await Timer(300, "ns")

    # Private read
    # Should read two byte as the target keeps SDA high
    data = await tb.i3c_controller.i3c_read(0x50, 2)
    assert data == bytearray([0xAA, 0xBB])

    await tb.i3c_controller.i3c_write(0x50, [0xCC])

    # Should read one byte as the target terminates the read after one byte
    # (the target will reject the read as it has no new data to send)
    data = await tb.i3c_controller.i3c_read(0x50, 2)
    assert data == bytearray([0xCC])

    await Timer(300, "ns")

    # Legacy I2C Read
    data = await tb.i3c_controller.i3c_read(0x40, 2, mode=I3cXferMode.LEGACY_I2C)

    await Timer(300, "ns")

    # Broadcasted SETMRL CCC
    await tb.i3c_controller.i3c_ccc_write(0xA, broadcast_data=[0xAA, 0xBB])

    await Timer(300, "ns")

    # Directed SETMRL CCC @ 0x51 and 0x52
    await tb.i3c_controller.i3c_ccc_write(
        0x8A, directed_data=[(0x51, [0xAA, 0xBB]), (0x52, [0xFF, 0x00])]
    )

    await Timer(300, "ns")

    response = await tb.i3c_controller.i3c_ccc_read(0x8C, 0x51, 2)
    mrl_data = response[0][1]
    assert len(mrl_data) == 2

    # Dump memory
    tb.log.info("Dump target memory")
    tb.i3c_target._mem.dump()


async def test_simple_write_followed_by_read(dut, address, issued_data):
    """
    Issues write to the I3C Target model and verifies it via private read to the target.
    """
    tb = I3cTestbench(dut, tgt_address=address)

    await Timer(100, "ns")

    await tb.i3c_controller.i3c_write(address, issued_data)
    recv_data = await tb.i3c_controller.i3c_read(address, 1)

    assert recv_data == bytearray(
        issued_data
    ), f"Written {[hex(_) for _ in issued_data]} to the target device but read {recv_data}"


@cocotb.test()
async def test_simple_write_followed_by_read_odd(dut):
    await test_simple_write_followed_by_read(dut, 0x60, [0xA2])


@cocotb.test()
async def test_simple_write_followed_by_read_even(dut):
    await test_simple_write_followed_by_read(dut, 0x60, [0xAA])


async def test_read_write_seq(dut, target_address, test_seq):
    """
    Performs a sequence of private writes & reads without issuing the STOP condition.
    The STOP condition is issued at the very end of the test.

    `test_seq` is a list of pairs, where:
        * the first element is the `address` to which a transfer is issued
        * second is a` list of bytes` to be written.
    """
    tb = I3cTestbench(dut, tgt_address=target_address)

    await Timer(100, "ns")

    for addr, data in test_seq:
        await tb.i3c_controller.i3c_write(addr, data, stop=False)
        recv_data = await tb.i3c_controller.i3c_read(addr, len(data), stop=False)
        # Dump memory
        tb.log.info("Dump target memory")
        tb.i3c_target._mem.dump()
        if addr == target_address:
            assert recv_data == bytearray(
                data
            ), f"Written {[hex(_) for _ in data]} to the target device but read {recv_data}"

    await tb.i3c_controller.send_stop()


# Enable once Sr / P conditions are properly recognized after data blocks
@cocotb.test(skip=True)
async def test_read_write_seq_same_device(dut):
    test_seq = [(0x50, [0xAA]), (0x50, [0xBB]), (0x50, [0x2A]), (0x50, [0xA2]), (0x50, [0x00])]
    await test_read_write_seq(dut, 0x50, test_seq)


# Enable once Sr / P conditions are properly recognized after data blocks
@cocotb.test(skip=True)
async def test_read_write_seq_other_device(dut):
    test_seq = [(0x50, [0xAA]), (0x60, [0xBB]), (0x50, [0x2A]), (0x51, [0xA2]), (0x50, [0x00])]
    await test_read_write_seq(dut, 0x50, test_seq)


# Enable once Sr / P conditions are properly recognized after data blocks
@cocotb.test(skip=True)
async def test_read_write_seq_randomized(dut):
    data_len = 100
    devices_num = 4
    devices = [randint(0x10, 0xE0) for _ in range(devices_num)]
    seq = [(choice(devices), [randint(0, 255)]) for _ in range(data_len)]
    await test_read_write_seq(dut, choice(devices), seq)
