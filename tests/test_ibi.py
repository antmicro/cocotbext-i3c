import cocotb
from cocotb.triggers import Timer
from utils import I3cExecError, I3cTestbench


def add_target_bcr_mdb(tb, tgt_address, mdb_enabled):
    tb.i3c_controller.add_target(tgt_address)
    target_idx = tb.i3c_controller.get_target_idx_by_addr(tgt_address)
    if target_idx is None:
        raise I3cExecError(
            f"Failed to add Target (address: {hex(tgt_address)}) to Controller targets"
        )
    tb.i3c_controller.targets[target_idx].set_bcr_fields(ibi_payload=mdb_enabled)


@cocotb.test()
async def test_simple_ibi(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)
    add_target_bcr_mdb(tb, tgt_address, mdb_enabled=False)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi()
    await Timer(100, "ns")


@cocotb.test()
async def test_simple_ibi_mdb(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)
    add_target_bcr_mdb(tb, tgt_address, mdb_enabled=True)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=0xAB)
    await Timer(100, "ns")


@cocotb.test()
async def test_simple_ibi_data(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)
    add_target_bcr_mdb(tb, tgt_address, mdb_enabled=True)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=0x19, data=bytearray([0x81, 0x20, 0x30, 0x40]))
    await Timer(100, "ns")


@cocotb.test()
async def test_simple_ibi_data_no_mdb(dut):
    tgt_address = 0x55
    tb = I3cTestbench(dut, tgt_address)
    add_target_bcr_mdb(tb, tgt_address, mdb_enabled=False)

    await Timer(100, "ns")
    await tb.i3c_target.send_ibi(mdb=None, data=bytearray([0x01]))
    await Timer(100, "ns")
