import cocotb
from cocotb.triggers import Timer
from utils import I3cTestbench


@cocotb.test()
async def test_simple_ibi(dut):
    tb = I3cTestbench(dut, tgt_address=0x2A)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi()
    await Timer(100, "ns")

@cocotb.test()
async def test_simple_ibi_mdb(dut):
    tb = I3cTestbench(dut, tgt_address=0x2A)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=0xab)
    await Timer(100, "ns")

@cocotb.test()
async def test_simple_ibi_data(dut):
    tb = I3cTestbench(dut, tgt_address=0x2A)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=0x19, data=bytearray([0x81, 0x20, 0x30, 0x40]))
    await Timer(100, "ns")
