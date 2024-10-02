import cocotb
from cocotb.triggers import Timer
from utils import I3cTestbench


@cocotb.test()
async def test_simple_ibi(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)

    target = tb.i3c_controller.add_target(tgt_address)
    target.set_bcr_fields(ibi_payload=False)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi()
    await Timer(100, "ns")


@cocotb.test()
async def test_simple_ibi_mdb(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)

    target = tb.i3c_controller.add_target(tgt_address)
    target.set_bcr_fields(ibi_payload=True)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=0xAB)
    await Timer(100, "ns")


@cocotb.test()
async def test_simple_ibi_data(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)

    target = tb.i3c_controller.add_target(tgt_address)
    target.set_bcr_fields(ibi_payload=True)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=0x19, data=bytearray([0x81, 0x20, 0x30, 0x40]))
    await Timer(100, "ns")


@cocotb.test()
async def test_simple_ibi_data_no_mdb(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)

    target = tb.i3c_controller.add_target(tgt_address)
    target.set_bcr_fields(ibi_payload=False)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=None, data=bytearray([0x01]))
    await Timer(100, "ns")
