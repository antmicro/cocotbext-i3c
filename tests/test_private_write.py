from random import randint

import cocotb
from cocotb.triggers import Timer
from utils import I3cExecError, I3cTestbench


async def test_simple_write(dut, address, data, len):
    """
    Issues a private write to the I3C Target model and verifies read data
    directly from the memory model.
    """
    tb = I3cTestbench(dut, tgt_address=address)

    await Timer(100, "ns")
    await tb.i3c_controller.i3c_write(address, data)
    await Timer(100, "ns")

    target_memory_slice = tb.i3c_target._mem._read_mem(0, len)

    # Dump memory for more insight
    if data != target_memory_slice:
        tb.log.info("Dump target memory")
        tb.i3c_target._mem.dump()
        raise I3cExecError(
            f"Written {[hex(_) for _ in data]} "
            f"to the target device but read {target_memory_slice}"
        )


@cocotb.test()
async def test_simple_write_odd(dut):
    await test_simple_write(dut, 0x40, [0xA2], 1)


@cocotb.test()
async def test_simple_write_even(dut):
    await test_simple_write(dut, 0x50, [0xAA], 1)


@cocotb.test()
async def test_simple_write_zero(dut):
    await test_simple_write(dut, 0x40, [0x00 for _ in range(10)], 10)


@cocotb.test()
async def test_simple_write_all_bits_set(dut):
    await test_simple_write(dut, 0x40, [0xFF for _ in range(13)], 13)


@cocotb.test()
async def test_simple_write_long(dut):
    await test_simple_write(dut, 0x10, [0xC0, 0xFF, 0xE0, 0x1A, 0x40, 0xDE, 0xAD, 0xBE, 0xEF], 9)


@cocotb.test()
async def test_simple_write_randomized(dut):
    data_len = 100
    data = [randint(0, 255) for _ in range(data_len)]
    await test_simple_write(dut, 0x10, data, data_len)
