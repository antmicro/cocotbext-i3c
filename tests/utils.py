#!/usr/bin/env python

import logging

from cocotbext_i3c.i3c_controller import I3cController
from cocotbext_i3c.i3c_target import I3CTarget


class I3cExecError(Exception):
    pass


class I3cTestbench:
    def __init__(self, dut, tgt_address=0x50):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        self.i3c_target = I3CTarget(
            sda_i=dut.sda_o,
            sda_o=dut.sda_tgt_i,
            scl_i=dut.scl_o,
            scl_o=dut.scl_tgt_i,
            debug_state_o=dut.debug_state_target_i,
            debug_detected_header_o=dut.debug_detected_header_i,
            speed=12.5e6,
            address=tgt_address,
        )

        self.i3c_controller = I3cController(
            sda_i=dut.sda_o,
            sda_o=dut.sda_ctrl_i,
            scl_i=dut.scl_o,
            scl_o=dut.scl_ctrl_i,
            debug_state_o=dut.debug_state_controller_i,
            speed=12.5e6,
            silent=False,
        )
