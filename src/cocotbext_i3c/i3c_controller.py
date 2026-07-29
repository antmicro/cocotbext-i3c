"""
Copyright (c) 2024-2025 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

import logging
from enum import Enum
from typing import Any, Callable, Iterable, Optional, TypeVar, Union

import cocotb
from cocotb.handle import ModifiableObject
from cocotb.triggers import (
    Edge,
    Event,
    FallingEdge,
    First,
    NextTimeStep,
    RisingEdge,
    Timer,
)

from .common import (
    I3C_RSVD_BYTE,
    I3cControllerTimings,
    I3cPRResp,
    I3cPWResp,
    I3cState,
    I3cTargetResetAction,
    calculate_tbit,
    make_timer,
    report_config,
    with_timeout_event,
)
from .hdr_bt import calculate_hdr_crc16, calculate_hdr_crc32
from .hdr_ddr import calculate_hdr_crc5

_T = TypeVar("_T")


# Sentinel raised when write_addr_header detects an IBI during the 0x7E phase
class IbiArbitrationEvent(Exception):
    pass


# Max retries when IBI keeps firing during transfer Start->0x7E
MAX_IBI_RETRIES = 10


class I3cXferMode(Enum):
    PRIVATE = 0
    LEGACY_I2C = 1

    @property
    def name(self) -> str:
        match self:
            case I3cXferMode.PRIVATE:
                return "Private"
            case I3cXferMode.LEGACY_I2C:
                return "Legacy I2C"


class Target:
    addr: int
    bcr: int

    def __init__(self, addr: int):
        self.addr = addr
        self.bcr = 0

    def set_bcr_fields(
        self,
        max_data_speed_limitation: bool = None,
        ibi_req_capable: bool = None,
        ibi_payload: bool = None,
        offline_capable: bool = None,
        virtual_target_support: bool = None,
        advanced_capabilities: bool = None,
        device_role: int = None,
    ):
        bcr = self.bcr & 0xFF

        # Clear each provided field and set to new value
        if max_data_speed_limitation:
            bcr = bcr & (1 << 0) | (max_data_speed_limitation << 0)
        if ibi_req_capable:
            bcr = bcr & (1 << 1) | (ibi_req_capable << 1)
        if ibi_payload:
            bcr = bcr & (1 << 2) | (ibi_payload << 2)
        if offline_capable:
            bcr = bcr & (1 << 3) | (offline_capable << 3)
        if virtual_target_support:
            bcr = bcr & (1 << 4) | (virtual_target_support << 4)
        if advanced_capabilities:
            bcr = bcr & (1 << 5) | (advanced_capabilities << 5)
        if device_role:
            bcr = bcr & (1 << 6) | (device_role << 6)

        self.bcr = bcr


class I3cController:
    FULL_SPEED: float = 12.5e6

    def __init__(
        self,
        sda_i: ModifiableObject,
        sda_o: ModifiableObject,
        scl_i: ModifiableObject,
        scl_o: ModifiableObject,
        debug_state_o: Optional[ModifiableObject] = None,
        timings: Optional[I3cControllerTimings] = None,
        speed: float = FULL_SPEED,
        silent=True,
        *args,
        **kwargs,
    ) -> None:
        self.log = logging.getLogger(f"cocotb.{sda_o._path}")
        self.log.setLevel("DEBUG")
        self.sda_i = sda_i
        self.sda_o = sda_o
        self.scl_i = scl_i
        self.scl_o = scl_o
        self.debug_state_o = debug_state_o
        self.speed = speed

        self.silent = silent

        if timings is None:
            timings = I3cControllerTimings()
        self.timings = timings

        def at_least_tsupp(period_ns: float) -> float:
            return period_ns if period_ns > timings.tsupp else timings.tsupp

        self.thd = make_timer(timings.thd, speed)
        self.tdig_h = make_timer(timings.tdig_h, speed)
        self.tdig_h_minus_thd = make_timer(at_least_tsupp(timings.tdig_h - timings.thd), speed)
        self.tdig_l = make_timer(at_least_tsupp(timings.tdig_l), speed)
        self.tdig_l_minus_thd = make_timer(at_least_tsupp(timings.tdig_l - timings.thd), speed)
        self.tsu_od = make_timer(timings.tsu_od, speed)
        self.tcas = make_timer(timings.tcas, speed)
        self.tcbp = make_timer(timings.tcbp, speed)
        self.tcbsr = make_timer(timings.tcbsr, speed)
        self.tcbsr_half = make_timer(timings.tcbsr / 2, speed)
        self.tcasr = make_timer(timings.tcasr, speed)
        self.tfree = make_timer(timings.tfree, speed)
        self.tsco = make_timer(timings.tsco, speed)
        self.tsu_pp = make_timer(timings.tsu_od, speed)

        self.hold_data = False

        self.targets = []
        self.got_ibi = Event()

        super().__init__(*args, **kwargs)

        if self.sda_o is not None:
            self.sda_o.setimmediatevalue(1)
        if self.scl_o is not None:
            self.scl_o.setimmediatevalue(1)
        self._state_ = I3cState.FREE
        self._state = I3cState.FREE

        self.interpret_target_peripheral_reset_timing_ns: Callable[[int], int] = lambda _: 1e6
        self.interpret_target_whole_reset_timing_ns: Callable[[int], int] = lambda _: 1e9
        self.interpret_target_net_adapter_reset_timing_ns: Callable[[int], int] = lambda _: 1e12

        report_config(self.speed, timings, lambda x: self.log_info(x))

        self.monitor_enable = Event()
        self.monitor_enable.set()
        self.monitor_idle = Event()
        self.monitor = False
        if self.sda_i is not None and self.scl_i is not None:
            self.monitor = True
            cocotb.start_soon(self._run())

        self.nack_ibis = Event()
        self.max_ibi_data_len = 65536  # This is the max value that can be set.

        # IBI result state
        self.last_ibi_result = None
        self.ibi_nack_count = 0
        self._ibi_event = Event()
        # One-shot IBI chaining knobs (consumed after next IBI)
        self._ibi_chain_ccc = None
        self._ibi_chain_ccc_data = None
        self._ibi_chain_ccc_addr = None
        self._ibi_chain_write_addr = None
        self._ibi_chain_write_data = None
        self._ibi_skip_data = False
        self._ibi_max_data = None

    def log_info(self, *args):
        if self.silent:
            return
        self.log.info(*args)

    @property
    def bus_active(self) -> bool:
        return self._state is not I3cState.FREE

    @property
    def _state(self) -> I3cState:
        return self._state_

    @property
    def remaining_tlow(self) -> Timer:
        return self.tdig_l if not self.hold_data else self.tdig_l_minus_thd

    @property
    def remaining_thigh(self) -> Timer:
        return self.tdig_h if not self.hold_data else self.tdig_h_minus_thd

    @_state.setter
    def _state(self, value: I3cState) -> None:
        self._state_ = value
        if self.debug_state_o is not None:
            self.debug_state_o.setimmediatevalue(value)

    @property
    def scl(self) -> Any:
        return self.scl_i.value

    @scl.setter
    def scl(self, value: Any) -> None:
        self.scl_o.value = value

    @property
    def sda(self) -> Any:
        return self.sda_i.value

    @sda.setter
    def sda(self, value: Any) -> None:
        self.sda_o.value = value

    def set_ibi_chain_ccc(self, ccc, ccc_data=None, ccc_addr=None):
        """Configure: after next IBI ACK/NACK, chain Sr->CCC before STOP."""
        assert (
            self._ibi_chain_write_addr is None
        ), "Cannot set CCC chain while write chain is active"
        self._ibi_chain_ccc = ccc
        self._ibi_chain_ccc_data = ccc_data
        self._ibi_chain_ccc_addr = ccc_addr

    def set_ibi_chain_write(self, addr, data):
        """Configure: after next IBI ACK/NACK, chain Sr->Private Write before STOP."""
        assert self._ibi_chain_ccc is None, "Cannot set write chain while CCC chain is active"
        self._ibi_chain_write_addr = addr
        self._ibi_chain_write_data = data

    def set_ibi_skip_data(self, skip=True):
        """Configure: after next IBI ACK, skip MDB/data read (BCR[2]=0 test)."""
        self._ibi_skip_data = skip

    def set_ibi_max_data(self, max_len):
        """Configure: limit IBI data bytes after MDB (truncation test)."""
        self._ibi_max_data = max_len

    def clear_ibi_chain(self):
        """Clear all one-shot chaining configuration."""
        self._ibi_chain_ccc = None
        self._ibi_chain_ccc_data = None
        self._ibi_chain_ccc_addr = None
        self._ibi_chain_write_addr = None
        self._ibi_chain_write_data = None
        self._ibi_skip_data = False
        self._ibi_max_data = None

    def _consume_ibi_chain(self):
        """Snapshot and clear the one-shot knobs. Returns a dict."""
        chain = {
            "ccc": self._ibi_chain_ccc,
            "ccc_data": self._ibi_chain_ccc_data,
            "ccc_addr": self._ibi_chain_ccc_addr,
            "write_addr": self._ibi_chain_write_addr,
            "write_data": self._ibi_chain_write_data,
            "skip_data": self._ibi_skip_data,
            "max_data": self._ibi_max_data,
        }
        self.clear_ibi_chain()
        return chain

    def reset_ibi_state(self):
        """Reset IBI tracking state. Call before starting a new IBI test sequence."""
        self.last_ibi_result = None
        self.ibi_nack_count = 0
        self._ibi_event.clear()

    async def wait_for_ibi_event(self, timeout=None, units="us"):
        """Wait for any IBI event (ACK or NACK). Returns the full result dict.

        Args:
            timeout: Optional timeout value. If None, waits indefinitely.
            units: Time units for timeout (default: "us")

        Returns:
            dict with keys: addr, data (bytearray), ack (bool),
            ccc_response, chain_write_ack
        """
        if timeout is not None:
            from cocotb.triggers import with_timeout

            await with_timeout(self._ibi_event.wait(), timeout, units)
        else:
            await self._ibi_event.wait()
        self._ibi_event.clear()
        return self.last_ibi_result

    def add_target(self, addr):
        """
        Add a Target to the Controller targets. It can then be accessed with:
            ```
            target_idx = i3c_controller.get_target_idx_by_addr(addr)
            if target_idx is not None:
                i3c_controller.targets[target_idx].set_bcr_fields()
                target = i3c_controller.targets[target_idx]
                target_bcr = target.bcr
            ```
            or
            ```
            target = i3c_controller.add_target(addr)
            target_bcr = target.bcr
            ```
        """
        for t in self.targets:
            if t.addr == addr:
                raise Exception(
                    f"Targets with the same addresses are not supported yet (address: {addr})"
                )

        target = Target(addr)
        self.targets.append(target)

        return target

    def get_target_idx_by_addr(self, addr):
        """
        Returns target index in `self.targets` if target was found, returns `None` otherwise.
        """
        for i, t in enumerate(self.targets):
            if t.addr == addr:
                return i
        return None

    async def take_bus_control(self):
        if not self.monitor:
            return
        # Disable bus monitor and wait for bus to enter idle state
        self.monitor_enable.clear()
        if not self.monitor_idle.is_set():
            await self.monitor_idle.wait()

    def give_bus_control(self):
        if not self.monitor:
            return
        self.monitor_enable.set()

    async def _hold_data(self):
        if self.hold_data:
            await self.thd
        else:
            await Timer(10, "ps")

    async def check_start(self):
        if not (self.sda and self.scl):
            return None

        sda_falling_edge = FallingEdge(self.sda_i)
        scl_falling_edge = FallingEdge(self.scl_i)
        result = await with_timeout_event(
            self.monitor_enable,
            First(sda_falling_edge, scl_falling_edge),
            I3cControllerTimings().tcas,
        )

        if result != sda_falling_edge:
            return None

        sda_rising_edge = RisingEdge(self.sda_i)
        if await First(self.tcas, sda_rising_edge, scl_falling_edge) != self.tcas:
            # Timing requirement for SDA low has not been met
            return None
        self.scl = 0
        await self.tsu_od

        return I3cState.START

    def _ccc_addresses_for_def_byte(
        def_bytes: Iterable[tuple[int, _T]], merge: bool = True
    ) -> Iterable[tuple[_T, list[int]]]:
        if merge:
            merged: dict[_T, list[int]] = {}
            for address, def_byte in def_bytes:
                if merged.get(def_byte) is None:
                    merged[def_byte] = []
                merged[def_byte].append(address)

            for def_byte, addresses in merged.items():
                yield def_byte, addresses
        else:
            for addr, def_byte in def_bytes:
                yield def_byte, [addr]

    async def send_start(self, pull_scl_low: bool = True) -> None:
        if self.bus_active:
            clock_after_data_t = self.tcasr
            self._state = I3cState.RS
            if pull_scl_low:
                self.scl = 0
            await self.thd
            self.sda = 1
            await self.tdig_l_minus_thd
        else:
            clock_after_data_t = self.tcas
            self._state = I3cState.START

        self.sda = 1
        self.scl = 1
        # Open-drain address to Push-Pull data handoff procedure description
        # says the time of SCL held high can be as low as T_{DIG_H}, however
        # min(T_{CBSr}) + min(T{CASr}) > min(T_{DIG_H})
        await self.tcbsr

        self.sda = 0
        await clock_after_data_t
        if pull_scl_low:
            self.scl = 0
        await self.tcasr

        self.hold_data = False

    async def send_stop(self, pull_scl_low: bool = True) -> None:
        self.log_info("I3C: STOP")
        self._state = I3cState.STOP
        if not self.bus_active:
            return

        if pull_scl_low:
            self.scl = 0
        await self._hold_data()
        self.sda = 0
        await self.remaining_tlow
        self.scl = 1
        await self.tcbp
        self.sda = 1
        await self.tfree

        self._state = I3cState.FREE
        self.hold_data = False

    async def send_hdr_exit(self) -> None:
        self.log_info("I3C: HDR exit")
        await self.take_bus_control()
        self._state = I3cState.FREE
        self.scl = 0
        self.sda = 1
        for _ in range(3):
            await self.tdig_h
            self.sda = 0
            await self.tdig_l
            self.sda = 1
        await self.tdig_h
        self.sda = 0
        await self.send_stop()
        self.give_bus_control()

    async def send_hdr_rstart(self) -> None:
        self.log_info("I3C: HDR RStart")
        await self.take_bus_control()
        self._state = I3cState.FREE
        self.scl = 0
        self.sda = 1
        for _ in range(2):
            await self.tdig_h
            self.sda = 0
            await self.tdig_l
            self.sda = 1
        await self._hold_data()
        await self.remaining_tlow
        self.scl = 1
        await self.tdig_h
        self.scl = 0
        await self._hold_data()

    async def send_hdr_preamble(
        self, preamble_code: Iterable[int] = [0, 1], stop_on_mismath: bool = False
    ) -> int:
        """Send HDR preamble pattern (alternating 0/1 on SDA while toggling SCL)."""
        self.log_info("I3C: HDR Preamble")
        drive = True
        return_value = 0
        # Preamble: 2 bits alternating pattern
        for bit in preamble_code:
            scl = self.scl
            if drive:
                self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            return_value = return_value << 1 | self.sda
            self.scl = 0 if scl else 1
            if self.sda != bit & 1 and stop_on_mismath:
                drive = False
                self.sda = 1
            await self._hold_data()
        # Release SDA
        self.sda = 1
        return return_value

    async def send_hdr_ddr_word(self, word: int, num_bits: int) -> None:
        """
        Send a word in HDR-DDR mode, MSB first.

        Args:
            word: Data word to send
            num_bits: Number of bits to send
        """
        for i in range(num_bits - 1, -1, -1):
            bit = (word >> i) & 1
            scl = self.scl
            self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            self.scl = 0 if scl else 1
            await self._hold_data()

    async def recv_hdr_ddr_word(self, num_bits: int) -> int:
        """
        Receive a word in HDR-DDR mode, MSB first.

        Args:
            num_bits: Number of bits to receive
        """
        word = 0
        for i in range(num_bits - 1, -1, -1):
            word <<= 1
            scl = self.scl
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            word |= self.sda & 1
            self.scl = 0 if scl else 1
            await self._hold_data()
        return word

    def calc_hdr_parity(self, data: int) -> int:
        """
        Calculates HDR parity value based on data

        Args:
            data: Source data to calculate parity
        """
        data = (data ^ (data >> 8)) & 0xFF
        data = (data ^ (data >> 4)) & 0xF
        data = (data ^ (data >> 2)) & 0x3
        data = data ^ 1
        return data

    async def send_hdr_parity(self, data: int) -> None:
        """
        Send HDR-DDR parity bits

        Args:
            data: Source data to calculate parity
        """
        await self.send_hdr_ddr_word(self.calc_hdr_parity(data), 2)

    async def send_hdr_ddr_write(
        self, addr: int, data: Iterable[int], send_CRC_on_termination: bool = False
    ) -> I3cPWResp:
        """
        Send HDR-DDR write transaction.

        Args:
            addr: 7-bit target address
            data: Data words to write
            send_CRC_on_termination: send CRC after write termination

        Returns:
            I3cPWResp
        """
        await self.take_bus_control()
        self.log_info(f"I3C: HDR-DDR Write to {hex(addr)}, data: {list(data)}")

        sent_words = 0
        nack = False
        premature_termination = False
        # Send preamble
        _ = await self.send_hdr_preamble()

        # Build command word (16 bits for HDR-DDR)
        # Bit [15]: R/W# (0 = Write)
        # Bits [14:8]: Command code (0x00 for standard write)
        # Bits [7:1]: Address (7 bits)
        # Bits [0]: Reserved/parity-bit
        cmd_word = (0 << 8) | (addr << 1) | 0x00
        parity = self.calc_hdr_parity(cmd_word) & 1
        cmd_word |= (parity ^ 1) & 1

        # Send command word
        await self.send_hdr_ddr_word(cmd_word, 16)
        await self.send_hdr_parity(cmd_word)

        preamble = await self.send_hdr_preamble([1, 1])
        if preamble == 0x3:
            self.log_info("I3C: HDR-DDR Write NACKed")
            nack = True
        elif preamble == 0x2:
            # Send data bytes with parity
            for i, word in enumerate(data):
                await self.send_hdr_ddr_word(word, 16)
                sent_words += 1
                # Send parity bit
                parity = self.calc_hdr_parity(word)
                await self.send_hdr_ddr_word(parity, 2)
                if i + 1 < len(data):
                    preamble = await self.send_hdr_preamble([1, 1])
                    if preamble == 0x2:  # NACK
                        premature_termination = True
                        break

            if not premature_termination or send_CRC_on_termination:
                # Calculate and send CRC-5
                crc_data = cmd_word.to_bytes(2, "big")
                for dat in data:
                    crc_data += dat.to_bytes(2, "big")
                crc = calculate_hdr_crc5(crc_data)
                crc = (crc << 1) | 1
                if not premature_termination:
                    _ = await self.send_hdr_preamble()
                await self.send_hdr_ddr_word(0xC, 4)
                await self.send_hdr_ddr_word(crc, 6)
        else:
            self.log_error(f"Unknown preamble value: {bin(preamble)}")

        self.give_bus_control()
        if nack:
            self.log_info("I3C: HDR-DDR Write failed")
        else:
            self.log_info(f"I3C: HDR-DDR Write complete, sent {sent_words} words")
        return I3cPWResp(nack, sent_words)

    async def send_hdr_ddr_read(
        self, addr: int, recv_CRC_on_termination: bool = False, interrupt: int = -1
    ) -> I3cPRResp:
        """
        Send HDR-DDR read transaction.

        Args:
            addr: 7-bit target address
            count: Number of words to read
            recv_CRC_on_termination: expect CRC on termination
            interrupt: Number of the data word that will be interrupted

        Returns:
            I3cPRResp
        """
        await self.take_bus_control()
        self.log_info(f"I3C: HDR-DDR Read from {hex(addr)}")

        nack = False
        premature_termination = False
        # Send preamble
        _ = await self.send_hdr_preamble()

        # Build command word (16 bits for HDR-DDR)
        # Bit [15]: R/W# (1 = Read)
        # Bits [14:8]: Command code (0x00 for standard write)
        # Bits [7:1]: Address (7 bits)
        # Bits [0]: Reserved/parity-bit
        cmd_word = (0x80 << 8) | (addr << 1) | 0x00

        parity = self.calc_hdr_parity(cmd_word) & 1
        cmd_word |= (parity ^ 1) & 1
        # Send command word
        await self.send_hdr_ddr_word(cmd_word, 16)
        await self.send_hdr_parity(cmd_word)

        data = []
        preamble = await self.send_hdr_preamble([1, 1])
        if preamble == 0x3:
            self.log_info("I3C: HDR-DDR Read NACKed")
            nack = True
        elif preamble == 0x2:
            # Read data bytes from target
            while True:
                word = await self.recv_hdr_ddr_word(16)
                data.append(word)
                parity = self.calc_hdr_parity(word)
                parity_recv = await self.recv_hdr_ddr_word(2)
                if parity != parity_recv:
                    self.log.error(f"HDR-DDR Read: Parity error on byte {hex(word)}")
                response = [1, 1]
                if interrupt == 0:
                    # Interrupt read operation
                    response = [1, 0]
                preamble = await self.send_hdr_preamble(response, stop_on_mismath=True)
                if preamble == 0x2:
                    premature_termination = True
                    break
                elif preamble == 0x1:
                    break

            if not premature_termination or recv_CRC_on_termination:
                crc_token = await self.recv_hdr_ddr_word(4)
                if crc_token != 0xC:
                    self.log.error(
                        f"HDR-DDR Read: CRC token error - received {hex(crc_token)}, expected 0xC"
                    )
                # Read CRC-5
                crc_received = await self.recv_hdr_ddr_word(6)
                crc_received >>= 1
                # Verify CRC
                crc_data = cmd_word.to_bytes(2, "big")
                for dat in data:
                    crc_data += dat.to_bytes(2, "big")
                crc_expected = calculate_hdr_crc5(crc_data)
                if crc_received != crc_expected:
                    self.log.error(
                        f"HDR-DDR Read: CRC error - received {hex(crc_received)}, expected {hex(crc_expected)}"
                    )
        else:
            self.log_error(f"Unknown preamble value: {bin(preamble)}")

        if nack:
            self.log_info("I3C: HDR-DDR Read failed")
        else:
            self.log_info(f"I3C: HDR-DDR Read complete - received {len(data)} words")

        self.give_bus_control()
        return I3cPRResp(nack, data)

    async def send_hdr_bt_word(self, word: int, num_bits: int) -> None:
        """
        Send a word in HDR-BT mode (DDR), MSB first.

        Args:
            word: Data word to send
            num_bits: Number of bits to send
        """
        for i in range(num_bits):
            bit = (word >> i) & 1
            scl = self.scl
            self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            self.scl = 0 if scl else 1
            await self._hold_data()

    async def recv_hdr_bt_word(self, num_bits: int, target_SCL: bool = False) -> int:
        """
        Receive a word in HDR-BT mode (DDR), MSB first.

        Args:
            num_bits: Number of bits to receive
        """
        return_value = 0
        if not target_SCL:
            for i in range(num_bits):
                scl = self.scl
                if scl:
                    await self.remaining_thigh
                else:
                    await self.remaining_tlow
                return_value = return_value | (self.sda << i)
                self.scl = 0 if scl else 1
                await self._hold_data()
        else:
            for i in range(num_bits):
                await Edge(self.scl_i)
                return_value = return_value | (self.sda << i)
                await self._hold_data()
        return return_value

    def calc_bt_parity(self, data: Iterable[int]) -> int:
        """
        Calculates HDR parity value based on data

        Args:
            data: Source data to calculate parity
        """
        base = 0
        for dat in data:
            base ^= dat
        base = (base ^ (base >> 4)) & 0xF
        base = (base ^ (base >> 2)) & 0x3
        base = (base ^ 1) & 0x3
        return base

    async def send_hdr_bt_header(
        self,
        addr: int,
        cmd: Iterable[int],
        read: bool = False,
        use_CRC32: bool = False,
        target_SCL: bool = False,
        CCC_continuation: bool = False,
    ) -> None:
        # Build command word (48 bits for HDR-BT)
        # Bit  [0]: R/W#
        # Bits [7:1]: Address (7 bits)
        # Bits [15:8]: cmd0
        # Bits [23:16]: cmd1
        # Bits [31:24]: cmd2
        # Bits [39:32]: cmd3
        # Bit  [40]: R/W#
        # Bit  [41]: CRC32/CRC16#
        # Bit  [42]: Controller SCL/Target SCL# (for read only)
        # Bit  [43]: Normal Message/CCC continuation#
        # Bits [45:44]: Reserved 0
        # Bits [47:46]: Parity
        read = 1 if read else 0
        crc = 1 if use_CRC32 else 0
        scl_mode = 1 if read and target_SCL else 0
        ccc = 1 if CCC_continuation else 0
        cmd_word = (
            read
            | (addr & 0x7F) << 1
            | cmd[0] << 8
            | cmd[1] << 16
            | cmd[2] << 24
            | cmd[3] << 32
            | read << 40
            | crc << 41
            | scl_mode << 42
            | ccc << 43
            | 0x0 << 44
        )
        parity = self.calc_bt_parity(cmd_word)
        cmd_word |= parity << 46
        await self.send_hdr_bt_word(cmd_word, 48)

    async def send_hdr_bt_header_transition(self, allow_delay_blocks: bool = False) -> int:
        # Build transition byte (8 bits for HDR-BT)
        allow_delay_blocks = 0 if allow_delay_blocks else 1
        word = 0x3 | allow_delay_blocks << 2 | 0x00 << 3
        return_value = 0
        for i in range(8):
            bit = (word >> i) & 1
            scl = self.scl
            self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            if i < 2:
                return_value = return_value | (self.sda << i)
            self.scl = 0 if scl else 1
            await self._hold_data()
        return return_value

    async def send_hdr_bt_transition_control(
        self, last_block: bool = False, data_words: int = 1
    ) -> int:
        # Build transition byte (8 bits for HDR-BT)
        last_block = 1 if last_block else 0
        if not last_block:
            data_words = 0
        else:
            data_words -= 1
        data = last_block << 2 | (data_words & 0xF) << 4
        parity = self.calc_bt_parity([data])
        parity = ((parity >> 1) ^ parity) & 1
        data |= parity << 3
        data |= 0x3

        return_value = 0
        for i in range(8):
            bit = (data >> i) & 1
            scl = self.scl
            self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            return_value = return_value | (self.sda << i)
            self.scl = 0 if scl else 1
            await self._hold_data()
        return return_value

    async def recv_hdr_bt_transition_control(
        self, target_SCL: bool = False, discard_transfer: bool = False
    ) -> int:
        # Build transition byte (8 bits for HDR-BT)
        word = 0xFF if not discard_transfer else 0xFD
        return_value = 0
        if not target_SCL:
            for i in range(8):
                bit = (word >> i) & 1
                scl = self.scl
                self.sda = bit
                if scl:
                    await self.remaining_thigh
                else:
                    await self.remaining_tlow
                return_value = return_value | (self.sda << i)
                self.scl = 0 if scl else 1
                await self._hold_data()
        else:
            for i in range(8):
                bit = (word >> i) & 1
                self.sda = bit
                await Edge(self.scl_i)
                return_value = return_value | (self.sda << i)
                await self._hold_data()
        return return_value

    async def send_hdr_bt_crc_control(
        self, use_CRC32: bool = False, terminated: bool = False
    ) -> None:
        use_CRC32 = 1 if use_CRC32 else 0
        terminated = 1 if terminated else 0
        ctrl_data = 0x0 | use_CRC32 << 5 | terminated << 6
        parity = self.calc_bt_parity([ctrl_data])
        parity = ((parity >> 1) ^ parity) & 1
        ctrl_data |= parity << 3
        await self.send_hdr_bt_word(ctrl_data, 8)

    async def send_hdr_bt_transition_verify(self) -> int:
        word = 0x3
        return_value = 0
        for i in range(8):
            bit = (word >> i) & 1
            scl = self.scl
            self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            if i < 2:
                return_value = return_value | (self.sda << i)
            self.scl = 0 if scl else 1
            await self._hold_data()
        return return_value

    async def recv_hdr_bt_transition_verify(
        self, accepted: bool = True, target_SCL: bool = False
    ) -> int:
        accepted = 0 if accepted else 1
        word = 1 | accepted << 1
        return_value = 0
        if not target_SCL:
            scl = self.scl
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            return_value = return_value | (self.sda << 0)
            self.scl = 0 if scl else 1
            await self._hold_data()
        else:
            # Handle SCL handoff
            await Edge(self.scl_i)
            return_value = return_value | (self.sda << 0)
            await self._hold_data()
        for i in range(1, 8):
            bit = (word >> i) & 1
            scl = self.scl
            self.sda = bit
            if scl:
                await self.remaining_thigh
            else:
                await self.remaining_tlow
            return_value = return_value | (self.sda << i)
            self.scl = 0 if scl else 1
            await self._hold_data()
        return return_value

    async def send_hdr_bt_write(
        self,
        addr: int,
        data: Iterable[int],
        use_CRC32: bool = False,
        command: Optional[Iterable[int]] = None,
    ) -> I3cPWResp:
        """
        Send HDR-BT write transaction using DDR bulk transfer.

        Args:
            addr: 7-bit target address
            data: Data words to write
            use_CRC32: use CRC32 instead of CRC16, default CRC16
            command: bytes to fill cmd0-cmd3, default all 0s

        Returns:
            I3cPWResp
        """
        await self.take_bus_control()
        self.log_info(f"I3C: HDR-BT Write to {hex(addr)}, data: {list(data)}")

        # Send header
        command = [0] * 4 if command is None else command
        await self.send_hdr_bt_header(addr, command, use_CRC32=use_CRC32)
        transition = await self.send_hdr_bt_header_transition()

        if transition == 0x3:
            self.log_info(f"I3C: HDR-BT Write to {hex(addr)} rejected")
            self.give_bus_control()
            return I3cPWResp(True, 0)

        transfered_words = 0
        terminated = False
        crc_values = []
        data_packets = [data[i : i + 16] for i in range(0, len(data), 16)]
        for i, data_packet in enumerate(data_packets):
            last = i == (len(data_packets) - 1)
            ctrl_pkt = await self.send_hdr_bt_transition_control(
                last=last, data_words=len(data_packet)
            )
            crc_values.append(ctrl_pkt)
            if (ctrl_pkt & 0x2) == 0:  # NACK from target
                terminated = True
                break
            for j in range(16):
                data = 0
                if j < len(data_packet):
                    transfered_words += 1
                    data = data_packet[j]
                    crc_values.append(data.to_bytes(2))
                await self.send_hdr_bt_word(data, 16)

        crc_values = bytearray(crc_values)
        await self.send_hdr_bt_crc_control(use_CRC32=use_CRC32, terminated=terminated)
        crc_value = (
            calculate_hdr_crc32(crc_values) if use_CRC32 else calculate_hdr_crc16(crc_values)
        )
        await self.send_hdr_bt_word(crc_value, 32)
        success = await self.send_hdr_bt_transition_verify()
        success = True if (success & 2) == 0 else False

        self.give_bus_control()
        if success:
            self.log_info("I3C: HDR-BT Write complete")
        else:
            self.log_info("I3C: HDR-BT Write failed")
        return I3cPWResp(not success, transfered_words)

    async def send_hdr_bt_read(
        self,
        addr: int,
        interrupt: int = -1,
        use_CRC32: bool = False,
        command: Optional[Iterable[int]] = None,
        target_SCL: bool = False,
        allow_delay_blocks: bool = False,
    ) -> I3cPRResp:
        """
        Send HDR-BT read transaction using DDR bulk transfer.

        Args:
            addr: 7-bit target address
            interrupt: Number of the data block that will be interrupted
            use_CRC32: use CRC32 instead of CRC16, default CRC16
            command: bytes to fill cmd0-cmd3, default all 0s
            target_SCL: pass SCL control to target, default off
            allow_delay_blocks: allow for delay blocks, default off

        Returns:
            I3cPRResp
        """
        await self.take_bus_control()
        self.log_info(f"I3C: HDR-BT Read from {hex(addr)}")

        # Send header
        command = [0] * 4 if command is None else command
        await self.send_hdr_bt_header(
            addr, command, read=True, use_CRC32=use_CRC32, target_SCL=target_SCL
        )
        transition = await self.send_hdr_bt_header_transition(allow_delay_blocks=allow_delay_blocks)

        if transition == 0x3:
            self.log_info(f"I3C: HDR-BT Read from {hex(addr)} rejected")
            self.give_bus_control()
            return I3cPRResp(True, [])

        terminated = False
        finished = False
        crc_values = []
        data_packets = []
        while not finished:
            ctrl_pkt = await self.recv_hdr_bt_transition_verify(
                target_SCL=target_SCL, discard_transfer=(interrupt == 0)
            )
            crc_values.append(ctrl_pkt)

            if interrupt == 0:
                terminated = True
                break
            block_delay = (ctrl_pkt & 0x10) != 0
            last = (ctrl_pkt & 0x4) != 0
            last_count = ((ctrl_pkt & 0xF0) >> 4) + 1
            finished = last
            if not block_delay:
                interrupt -= 1
            for i in range(16):
                word = await self.recv_hdr_bt_word(16, target_SCL=target_SCL)
                if not block_delay and (not last or i < last_count):
                    data_packets.append(word)
                    crc_values.append(word.to_bytes(2))

        crc_values = bytearray(crc_values)
        expected_crc_value = (
            calculate_hdr_crc32(crc_values) if use_CRC32 else calculate_hdr_crc16(crc_values)
        )
        crc_ctrl = await self.recv_hdr_bt_word(8, target_SCL=target_SCL)
        crc_value = await self.recv_hdr_bt_word(32, target_SCL=target_SCL)

        use_CRC32 = 1 if use_CRC32 else 0
        accepted = ((crc_ctrl >> 5) & 1) == use_CRC32
        terminated = 1 if terminated else 0
        accepted &= ((crc_ctrl >> 6) & 1) == terminated
        accepted &= expected_crc_value == crc_value
        success = await self.recv_hdr_bt_transition_verify(accepted=accepted, target_SCL=target_SCL)
        success = True if (success & 2) == 0 else False

        self.give_bus_control()
        if success:
            self.log_info("I3C: HDR-BT Read complete - received {len(data_packets)} words")
        else:
            self.log_info("I3C: HDR-BT Read failed")
        return I3cPRResp(not success, data_packets)

    async def send_target_reset_pattern(self) -> None:
        await self.take_bus_control()
        self._state = I3cState.TARGET_RESET

        sda = 1
        self.sda = sda
        self.scl = 0
        for _ in range(14):
            await self.tdig_h
            sda = 0 if (sda != 0) else 1
            self.sda = sda
        await self.tdig_h
        self.scl = 1

        await self.tdig_h
        await self.send_start(pull_scl_low=False)
        await self.send_stop(pull_scl_low=False)

        # TODO: Send start and I3C address header?
        self.give_bus_control()

    async def target_reset(
        self,
        reset_actions: Union[
            Iterable[tuple[int, I3cTargetResetAction]], I3cTargetResetAction, None
        ] = None,
        query_timings: Union[bool, Iterable[int]] = False,
        assumed_default_action: I3cTargetResetAction = I3cTargetResetAction.RESET_PERIPHERAL_ONLY,
        merge_ccc_actions=True,
    ) -> None:
        """
        Several scenarios are supported.
        Reset with default configuration, eg:
        ```
            await i3c_controller.target_reset()
        ```

        Reset with broadcasted configuration, eg:
        ```
            await i3c_controller.target_reset(
                reset_actions=I3cTargetResetAction.RESET_PERIPHERAL_ONLY
            )
        ```
        Reset with direct configuration for given addresses, eg:
        ```
            await tb.i3c_controller.target_reset(
                reset_actions=[
                    (0x20, I3cTargetResetAction.RESET_PERIPHERAL_ONLY),
                    (0x21, I3cTargetResetAction.RESET_WHOLE_TARGET),
                    (0x22, I3cTargetResetAction.RESET_WHOLE_TARGET)
                ]
            )
        ```

        Same direct configurations get merged into a single CCC by default, but it can be disabled
        with `merge_ccc_actions=False`.
        """
        if reset_actions is None:
            await self.send_start()

        last_ccc = "none"

        # Set up reset actions

        match reset_actions:
            case I3cTargetResetAction():
                # Broadcast RSTACT
                await self.i3c_ccc_write(ccc=0x2A, defining_byte=reset_actions, stop=False)
                last_ccc = "broadcast"
            case None:
                pass
            case _:  # Assume iterable
                # Directed RSTACT
                for reset_action, addresses in I3cController._ccc_addresses_for_def_byte(
                    def_bytes=reset_actions, merge=merge_ccc_actions
                ):
                    print(f"Reset action {reset_action} for {addresses}")
                    await self.i3c_ccc_write(
                        ccc=0x9A,
                        defining_byte=reset_action,
                        directed_data=map(lambda addr: (addr, []), addresses),
                        stop=False,
                    )
                    last_ccc = "direct"

        queries: list[tuple[int, int]] = []

        def add_timing_query_for_reset_action(addr: int, action: I3cTargetResetAction):
            match action:
                case I3cTargetResetAction.NO_RESET:
                    pass
                case I3cTargetResetAction.RESET_PERIPHERAL_ONLY:
                    queries.append((addr, 0x81))
                case I3cTargetResetAction.RESET_WHOLE_TARGET:
                    queries.append((addr, 0x82))
                case I3cTargetResetAction.DEBUG_NETWORK_ADAPTER_RESET:
                    queries.append((addr, 0x83))
                case _:
                    raise RuntimeError("Unsupported reset action for timing query: " f"`{action}`")

        # Prepare RSTACT queries

        match query_timings, reset_actions:
            case False, _:
                pass
            case True, I3cTargetResetAction():
                raise RuntimeError(
                    "query_timings can't be used without " "specifying reset targets"
                )
            case True, _:  # Assume Iterable
                for addr, action in reset_actions:
                    add_timing_query_for_reset_action(addr, action)
            case _, _:  # Assume Iterable
                addr_actions: dict[int, list[I3cTargetResetAction]] = {}
                for address, action in reset_actions:
                    if addr_actions.get(address) is None:
                        addr_actions[address] = []
                    addr_actions[address].append(action)

                for address in query_timings:
                    actions = addr_actions.get(address)
                    if actions is not None:
                        for action in actions:
                            add_timing_query_for_reset_action(address, action)
                    else:
                        add_timing_query_for_reset_action(address, assumed_default_action)
            case _, I3cTargetResetAction():  # Assume Iterable
                for address in query_timings:
                    add_timing_query_for_reset_action(address, reset_actions)
            case _, None:  # Assume Iterable
                for address in query_timings:
                    add_timing_query_for_reset_action(address, assumed_default_action)

        # Query and calculate reset time

        max_timing = 0
        # TODO: expand semantics of i3c_ccc_read to allow querying multiple addresses
        # within a single CCC
        for address, def_byte in queries:
            # TODO: Handle NACKs
            timing_v = await self.i3c_ccc_read(
                ccc=0x2A, addr=address, count=1, defining_byte=def_byte
            )[0]

            last_ccc = "direct"

            timing_ns = 0
            match def_byte:
                case 0x81:
                    timing_ns = self.interpret_target_peripheral_reset_timing_ns(timing_v)
                case 0x82:
                    timing_ns = self.interpret_target_whole_reset_timing_ns(timing_v)
                case 0x83:
                    timing_ns = self.interpret_target_net_adapter_reset_timing_ns(timing_v)
            if timing_ns > max_timing:
                max_timing = timing_ns

        # Finish sending CCCs without closing the frame
        await self.take_bus_control()
        match last_ccc:
            case "none":
                pass
            case "broadcast":
                await self.send_start()
            case "direct":
                await self.send_start()
                await self.write_addr_header(I3C_RSVD_BYTE)
                await self.send_start()
        self.give_bus_control()

        await self.send_target_reset_pattern()

        if max_timing != 0:
            await Timer(max_timing, "ns")

    async def send_bit(self, b: bool) -> None:
        if not self.bus_active:
            self.send_start()

        self.scl = 0
        await self._hold_data()
        self.sda = bool(b)
        await self.remaining_tlow
        self.scl = 1
        await self.tdig_h
        self.hold_data = True

    async def send_bit_arb(self, b: bool) -> tuple:
        """Drive a bit in open-drain and read back the bus value.

        Per I3C spec Sec.5.1.2.2.1: if the device drives Hi-Z (1) but reads
        Low (0), another device is driving -> arbitration lost.

        Returns:
            (arb_ok, bus_value): arb_ok is False if we lost arbitration.
        """
        self.scl = 0
        await self._hold_data()
        self.sda = b
        await self.remaining_tlow
        # Sample bus before raising SCL (same timing as recv_bit)
        bus_val = bool(self.sda) if self.sda_i is not None else b
        self.scl = 1
        await self.tdig_h
        self.hold_data = True
        # Arb lost when we released (drove 1) but bus is low (0)
        arb_ok = not (b and not bus_val)
        return (arb_ok, bus_val)

    async def recv_bit(self) -> bool:
        if not self.bus_active:
            self.send_start()

        self.scl = 0
        await self._hold_data()
        self.sda = 1
        await self.remaining_tlow
        if self.sda_i is None:
            b = False
        else:
            b = bool(self.sda)
        self.scl = 1
        await self.tdig_h
        self.hold_data = False

        return b

    async def recv_bit_od(self) -> bool:
        if not self.bus_active:
            self.send_start()

        self.scl = 0
        self.sda = 1
        # We don't hold the data here, because it's on the target to pull it down
        # after the required amount of time
        await self.tdig_l
        if self.sda_i is None:
            b = False
        else:
            b = bool(self.sda)
        self.scl = 1
        await self.tdig_h
        self.hold_data = False

        return b

    async def recv_addr_ack(self) -> bool:
        assert self.bus_active

        self.scl = 0
        self.sda = 1
        # We don't hold the data here, because it's on the target to pull it down
        # after the required amount of time
        await self.tdig_l
        if self.sda_i is None:
            b = False
        else:
            b = bool(self.sda)

        # Take over driving in case of an ACK by target
        if not b:
            self.sda = 0
        self.scl = 1
        await self.tdig_h
        self.hold_data = False

        return b

    async def send_byte(self, b: int, addr: bool = False) -> bool:
        self._state = I3cState.ADDR if addr else I3cState.DATA_WR
        for i in range(8):
            await self.send_bit(b & (1 << 7 - i))
        self._state = I3cState.ACK

        # Expect an ACK after a write address
        if addr and not (b & 1):
            return await self.recv_addr_ack()
        else:
            return await self.recv_bit_od()

    async def send_byte_arb(self, b: int) -> tuple:
        """Drive a byte with per-bit arbitration check.

        On first arbitration loss, stops driving and switches to reading
        remaining bits so the bus stays synchronized.

        Returns:
            (arb_ok, bus_byte): arb_ok is False if arbitration was lost.
            bus_byte contains the actual byte seen on the bus.
        """
        self._state = I3cState.ADDR
        bus_byte = 0
        arb_ok = True
        for i in range(8):
            bit_val = bool(b & (1 << (7 - i)))
            if arb_ok:
                ok, bus_bit = await self.send_bit_arb(bit_val)
                if not ok:
                    arb_ok = False
                    self.log_info(
                        f"Arbitration lost at bit {i}: " f"drove {int(bit_val)}, bus={int(bus_bit)}"
                    )
            else:
                # Lost arbitration - just read remaining bits
                bus_bit = await self.recv_bit()
            bus_byte = (bus_byte << 1) | int(bus_bit)
        return (arb_ok, bus_byte)

    async def recv_byte(self, send_ack: bool) -> int:
        b = 0
        self._state = I3cState.DATA_RD
        for _ in range(8):
            b = (b << 1) | await self.recv_bit()
        self._state = I3cState.ACK
        # ACK is indicated by pulling SDA low
        ack = not send_ack
        await self.send_bit(ack)
        return b

    async def send_byte_tbit(self, b: int, inject_tbit_err: bool = False) -> None:
        self.log_info(f"Controller:::Send byte {b}")
        self._state = I3cState.DATA_WR
        for i in range(8):
            await self.send_bit(bool(b & (1 << (7 - i))))
        # Send T-Bit
        self._state = I3cState.TBIT_WR
        await self.send_bit(calculate_tbit(b, inject_tbit_err))

    async def tbit_eod(self, request_end: bool) -> bool:
        self.scl = 0
        await self.tdig_l
        eod = not bool(self.sda)
        # At this point target should set SDA to High-Z.
        self.scl = 1
        if eod:  # Target requests end-of-data
            self.sda = 0
            self.hold_data = False
            await self.tdig_h
            # This should be followed by a stop signal: self.send_stop
        elif request_end:  # Controller requests end-of-data
            # This is basically RS and should be followed by a stop signal: self.send_stop
            await self.tcbsr
            self.sda = 0
            await self.tcasr
        else:
            self.hold_data = False
            await self.tdig_h

        return eod

    async def recv_byte_t_bit(self, stop: bool) -> tuple[int, bool]:
        b = 0
        self._state = I3cState.DATA_RD
        for _ in range(8):
            b = (b << 1) | (1 if await self.recv_bit() else 0)
        self._state = I3cState.TBIT_RD
        tgt_eod = await self.tbit_eod(request_end=stop)
        return (b, tgt_eod | stop)

    async def write_addr_header(self, addr: int, read: bool = False) -> bool:
        # Handle IBI arbitration
        if addr == I3C_RSVD_BYTE and not read:
            self.log_info("Address Header:::Reserved I3C Address Header 0x%02x", addr)

            arb_ok, bus_byte = await self.send_byte_arb(addr << 1)
            if not arb_ok:
                # Bus byte is the IBI target's addr+RnW - handle the IBI
                self.log_info(f"IBI detected during 0x7E: bus_byte=0x{bus_byte:02X}")

                # Handle IBI without STOP - caller will retry with Sr
                # so the DUT sees a Repeated Start and won't re-arbitrate
                await self._handle_ibi(bus_byte, send_stop=False)
                raise IbiArbitrationEvent()

            # No arbitration conflict - receive ACK from target
            self._state = I3cState.ACK
            nack = await self.recv_addr_ack()
        else:
            if addr == I3C_RSVD_BYTE:
                self.log_info("Address Header:::Reserved I3C Address Header 0x%02x", addr)
            else:
                self.log_info(
                    "Address Header:::Address Header to device at I3C address 0x%02x", addr
                )
            nack = await self.send_byte((addr << 1) | (0 if not read else 1), addr=True)

        if nack:
            self.log_info("Address Header:::Got NACK")
        else:
            self.log_info("Address Header:::Got ACK")
        return not nack

    async def recv_until_eod_tbit(self, buf: bytearray, count: int, stop: bool = True) -> None:
        length = count if count else 1

        while length:
            length = (length - 1) if count else 1
            byte, tgt_stop = await self.recv_byte_t_bit(stop=stop and not length)
            buf.append(byte)
            if tgt_stop:
                return

    async def i3c_write(
        self,
        addr: int,
        data: Iterable[int],
        stop: bool = True,
        mode: I3cXferMode = I3cXferMode.PRIVATE,
        inject_tbit_err: bool = False,
        inject_tbit_err_on: Optional[int] = None,
        send_rsvd: bool = True,
        rstart: bool = False,
        take_bus_control: bool = True,
    ) -> I3cPWResp:
        """I3C Private Write transfer"""
        if take_bus_control:
            await self.take_bus_control()
        self.log_info(f"I3C: Write data ({mode.name}) {data} @ {hex(addr)}")

        for retry in range(MAX_IBI_RETRIES):
            try:
                if send_rsvd:
                    await self.send_start()
                    await self.write_addr_header(I3C_RSVD_BYTE)
                await self.send_start()

                ack = await self.write_addr_header(addr)
                if ack:
                    for i, d in enumerate(data):
                        match mode:
                            case I3cXferMode.PRIVATE:
                                do_tbit_err = (i == inject_tbit_err_on) or inject_tbit_err
                                await self.send_byte_tbit(d, do_tbit_err)
                            case I3cXferMode.LEGACY_I2C:
                                await self.send_byte(d)
                        self.log_info(f"I3C: wrote byte {hex(d)}, idx={i}")

                if rstart:
                    await self.send_start()
                elif stop:
                    await self.send_stop()

                self.give_bus_control()
                return I3cPWResp(not ack, len(data))
            except IbiArbitrationEvent:
                self.log_info(f"I3C Write: IBI handled, retrying ({retry + 1})")
                continue

        self.give_bus_control()
        raise RuntimeError(f"i3c_write: exceeded {MAX_IBI_RETRIES} IBI retries")

    async def i3c_read(
        self,
        addr: int,
        count: int,
        stop: bool = True,
        mode: I3cXferMode = I3cXferMode.PRIVATE,
        send_rsvd: bool = True,
    ) -> I3cPRResp:
        """I3C Private Read transfer"""
        await self.take_bus_control()
        data = bytearray()
        self.log_info(f"I3C: Read data ({mode.name}) @ {hex(addr)}")

        for retry in range(MAX_IBI_RETRIES):
            try:

                if send_rsvd:
                    await self.send_start()
                    await self.write_addr_header(I3C_RSVD_BYTE)
                await self.send_start()
                ack = await self.write_addr_header(addr, read=True)
                if ack:
                    match mode:
                        case I3cXferMode.PRIVATE:
                            await self.recv_until_eod_tbit(data, count)
                        case I3cXferMode.LEGACY_I2C:
                            for i in range(count):
                                send_ack = not (i == count - 1)
                                data.append(await self.recv_byte(send_ack))
                if stop:
                    await self.send_stop()

                self.give_bus_control()
                return I3cPRResp(not ack, data)
            except IbiArbitrationEvent:
                self.log_info(f"I3C Read: IBI handled, retrying ({retry + 1})")
                continue

        self.give_bus_control()
        raise RuntimeError(f"i3c_read: exceeded {MAX_IBI_RETRIES} IBI retries")

    async def i3c_ccc_write(
        self,
        ccc: int,
        broadcast_data: Optional[Iterable[int]] = None,
        directed_data: Optional[Iterable[tuple[int, Iterable[int]]]] = None,
        defining_byte: Optional[int] = None,
        stop: bool = True,
        pull_scl_low: bool = False,
    ) -> Iterable[bool]:
        """Issue CCC Write frame. For directed CCCs use an iterable of address-data tuples"""
        await self.take_bus_control()
        is_broadcast = ccc <= 0x7F

        log_data = broadcast_data if is_broadcast else directed_data
        if is_broadcast:
            self.log_info(f"I3C: CCC {hex(ccc)} WR (Broadcast): {log_data}")
        else:
            self.log_info(f"I3C: CCC {hex(ccc)} WR (Directed): {log_data}")

        for retry in range(MAX_IBI_RETRIES):
            try:
                acks = []

                await self.send_start()
                await self.write_addr_header(I3C_RSVD_BYTE)
                await self.send_byte_tbit(ccc)
                if defining_byte is not None:
                    await self.send_byte_tbit(defining_byte)

                if is_broadcast:
                    if broadcast_data is not None:
                        for byte in broadcast_data:
                            await self.send_byte_tbit(byte)
                else:
                    assert directed_data is not None

                    for addr, data in directed_data:
                        await self.send_start()
                        acks.append(await self.write_addr_header(addr))
                        for byte in data:
                            await self.send_byte_tbit(byte)

                if stop:
                    await self.send_stop()
                if pull_scl_low:
                    self.scl = 0
                    await self._hold_data()

                self.give_bus_control()
                return acks

            except IbiArbitrationEvent:
                self.log_info(f"I3C CCC Write: IBI handled, retrying ({retry + 1})")
                continue

        self.give_bus_control()
        raise RuntimeError(f"i3c_ccc_write: exceeded {MAX_IBI_RETRIES} IBI retries")

    async def i3c_ccc_read(
        self,
        ccc: int,
        addr: [int, Iterable[int]],
        count: int,
        defining_byte: Optional[int] = None,
        stop: bool = True,
    ) -> Iterable[tuple]:
        """
        Issue directed CCC Read frame. For multiple targets use address list.
        Returns a list of tuples with (ack, data) for each target address.
        """

        if isinstance(addr, int):
            addr = [addr]

        await self.take_bus_control()
        astr = " ".join([hex(a) for a in addr])
        self.log_info(f"I3C: CCC {hex(ccc)} RD (Directed @ {astr})")

        for retry in range(MAX_IBI_RETRIES):
            try:
                responses = []

                await self.send_start()
                await self.write_addr_header(I3C_RSVD_BYTE)
                await self.send_byte_tbit(ccc)
                if defining_byte is not None:
                    await self.send_byte_tbit(defining_byte)
                for a in addr:
                    await self.send_start()
                    ack = await self.write_addr_header(a, read=True)
                    data = bytearray()
                    await self.recv_until_eod_tbit(data, count, stop=False)
                    responses.append((ack, data))

                if stop:
                    await self.send_stop()

                self.give_bus_control()
                return responses

            except IbiArbitrationEvent:
                self.log_info(f"I3C CCC Read: IBI handled, retrying ({retry + 1})")
                continue

        self.give_bus_control()
        raise RuntimeError(f"i3c_ccc_read: exceeded {MAX_IBI_RETRIES} IBI retries")

    _DIRECTED_GET_CCCS = frozenset(
        {
            0x8B,  # GETMWL
            0x8C,  # GETMRL
            0x8D,  # GETPID
            0x8E,  # GETBCR
            0x8F,  # GETDCR
            0x90,  # GETSTATUS
            0x91,  # GETACCCR
            0x94,  # GETMXDS
            0x95,  # GETCAPS
            0x99,  # GETXTIME
        }
    )

    async def _handle_ibi(self, ibi_addr_byte: Optional[int] = None, send_stop: bool = True):
        """
        Receive and IBI from the target, support for MDB is determined from the `self.targets` list
        which should be configured by the testbench. If there is no entry for the target with an address
        received on the bus, assume that the BCR has no set values hence is equal to 0.
        """
        chain = self._consume_ibi_chain()

        # Accept/reject the interrupt by sending an ACK/NACK
        ack = not self.nack_ibis.is_set()

        if ibi_addr_byte is not None:
            addr = ibi_addr_byte >> 1
            # Send ACK/NACK
            self._state = I3cState.ACK
            # ACK = drive SDA low, NACK = release SDA (high)
            await self.send_bit(not ack)
        else:
            addr = await self.recv_byte(send_ack=ack) >> 1

        result = {
            "addr": addr,
            "data": bytearray(),
            "ack": ack,
            "ccc_response": None,
            "chain_write_ack": None,
        }

        max_data = chain["max_data"] if chain["max_data"] is not None else self.max_ibi_data_len

        # Receive IBI
        data = result["data"]
        if ack and not chain["skip_data"]:
            self.log.info(f"ACK-ed an IBI from 0x{addr:02X}")
            target_idx = self.get_target_idx_by_addr(addr)
            if target_idx is not None:
                self.targets[target_idx].set_bcr_fields()
                target = self.targets[target_idx]
                mdb_enabled = target.bcr & (1 << 2)
                if mdb_enabled:
                    await self.recv_until_eod_tbit(data, max_data + 1)
                    self.log.info(
                        f"IBI MDB: 0x{data[0]:02X}, data: ["
                        + " ".join([f"0x{d:02X}" for d in data[1:]])
                        + "]"
                    )
            else:
                self.log.warning(f"Target ({hex(addr)}) has no configured BCR, assuming BCR = 0")
        else:
            self.log.info(f"NACK-ed an IBI from 0x{addr:02X}")

        # Execute chained action (chains always include their own STOP)
        if chain["ccc"] is not None:
            await self._execute_chain_ccc(chain, result)
        elif chain["write_addr"] is not None:
            await self._execute_chain_write(chain, result)
        elif send_stop:
            await self.send_stop()

        # Update result tracking state
        self.last_ibi_result = result
        if ack:
            self.got_ibi.set(bytearray([addr]) + result["data"])
        else:
            self.ibi_nack_count += 1
        self._ibi_event.set()

    async def _execute_chain_ccc(self, chain, result):
        """Execute Sr->CCC chain after IBI handling."""
        await self.send_start()
        # Use base class send_byte for 0x7E after Sr (not arbitrable)
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_byte_tbit(chain["ccc"])
        if chain["ccc_data"] is not None:
            if chain["ccc_addr"] is not None:
                is_read = chain["ccc"] in self._DIRECTED_GET_CCCS
                await self.send_start()
                await self.write_addr_header(chain["ccc_addr"], read=is_read)
                if is_read:
                    resp_data = bytearray()
                    await self.recv_until_eod_tbit(resp_data, len(chain["ccc_data"]))
                    result["ccc_response"] = resp_data
                else:
                    for byte in chain["ccc_data"]:
                        await self.send_byte_tbit(byte)
            else:
                for byte in chain["ccc_data"]:
                    await self.send_byte_tbit(byte)
        await self.send_stop()

    async def _execute_chain_write(self, chain, result):
        """Execute Sr->Private Write chain after IBI handling."""
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_start()
        write_ack = await self.write_addr_header(chain["write_addr"])
        result["chain_write_ack"] = write_ack
        if write_ack and chain["write_data"]:
            for byte in chain["write_data"]:
                await self.send_byte_tbit(byte)
        await self.send_stop()

    def enable_ibi(self, enable):
        """
        Enables/disables ACK-ing IBIs
        """
        if enable:
            self.nack_ibis.clear()
        else:
            self.nack_ibis.set()

    def set_max_ibi_data_len(self, max_len):
        """
        Sets the maximum number of IBI data bytes that follow MDB. Beyond the
        given count the controller will terminate IBI with stop.
        """
        self.max_ibi_data_len = max_len

    async def wait_for_ibi(self):
        """
        Waits for an IBI. Returns its data
        """
        await self.got_ibi.wait()
        data = self.got_ibi.data  # Get data from the event
        self.got_ibi.clear()
        return data

    @staticmethod
    def _odd_parity(addr: int) -> int:
        """Compute odd parity over 7-bit address for ENTDAA address byte."""
        p = 1
        for i in range(7):
            p ^= (addr >> i) & 1
        return p

    async def i3c_entdaa(
        self,
        addrs_to_assign: list,
        inject_te3_parity: bool = False,
        inject_te4_invalid_rsvd: bool = False,
        stop_after_n_targets: int = None,
    ) -> list:
        """
        Execute ENTDAA from the controller side.

        Protocol:
          S + 7E/W + ACK -> 0x07 + T-bit
          For each target:
            Sr + 7E/R + ACK
            Read 64 bits (PID+BCR+DCR) via recv_bit_od
            Send address byte (addr<<1 | odd_parity) via send_byte
            Target ACKs
          P (STOP)

        Args:
            addrs_to_assign: List of 7-bit addresses to assign to targets.
            inject_te3_parity: Flip parity bit on address byte (TE3 error).
            inject_te4_invalid_rsvd: Send 7E/W instead of 7E/R after Sr (TE4).
            stop_after_n_targets: Issue STOP after N targets (early termination).

        Returns:
            List of dicts: {pid, bcr, dcr, addr, ack} per target slot.
        """
        await self.take_bus_control()
        results = []

        # Phase 1: S + 7E/W + ACK + ENTDAA CCC (0x07) + T-bit
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_byte_tbit(0x07)

        targets_done = 0
        for addr in addrs_to_assign:
            if stop_after_n_targets is not None and targets_done >= stop_after_n_targets:
                break

            # Phase 2: Sr + 7E/R (or 7E/W for TE4 injection)
            await self.send_start()
            if inject_te4_invalid_rsvd:
                ack = await self.write_addr_header(I3C_RSVD_BYTE, read=False)
            else:
                ack = await self.write_addr_header(I3C_RSVD_BYTE, read=True)

            if not ack:
                # NACK — no more targets to assign (or TE4 error)
                results.append({"pid": None, "bcr": None, "dcr": None, "addr": addr, "ack": False})
                break

            # Phase 3: Read 64 bits of device ID (PID[48] + BCR[8] + DCR[8])
            id_bits = 0
            for bit_idx in range(64):
                bit_val = await self.recv_bit_od()
                id_bits = (id_bits << 1) | int(bit_val)

            # Parse: PID = bits[63:16], BCR = bits[15:8], DCR = bits[7:0]
            pid = (id_bits >> 16) & 0xFFFFFFFFFFFF
            bcr = (id_bits >> 8) & 0xFF
            dcr = id_bits & 0xFF

            # Phase 4: Send address byte with parity
            parity = self._odd_parity(addr)
            if inject_te3_parity:
                parity ^= 1  # Flip parity to cause TE3
            addr_byte = (addr << 1) | parity

            nack = await self.send_byte(addr_byte)
            target_acked = not nack

            results.append({"pid": pid, "bcr": bcr, "dcr": dcr, "addr": addr, "ack": target_acked})
            targets_done += 1

        # Phase 5: STOP
        await self.send_stop()
        self.give_bus_control()
        return results

    async def send_target_reset_pattern_stress(
        self,
        num_transitions: int = 14,
        tdig_h_ns: Optional[float] = None,
        t_start_hold_ns: Optional[float] = None,
        t_stop_hold_ns: Optional[float] = None,
    ) -> None:
        """
        Send a target reset pattern with configurable parameters for stress testing.

        Per I3C spec (5.1.11.3):
        1. 14 SDA transitions while SCL is kept Low
        2. Repeated START
        3. STOP

        Args:
            num_transitions: Number of SDA transitions (14 for valid pattern)
            tdig_h_ns: Override timing for SDA transitions in nanoseconds.
                       If None, uses the controller's default tdig_h.
            t_start_hold_ns: Override hold time for START condition in nanoseconds.
                             If None, uses tdig_h_ns or controller default.
            t_stop_hold_ns: Override hold time for STOP condition in nanoseconds.
                            If None, uses tdig_h_ns or controller default.
        """
        await self.take_bus_control()
        self._state = I3cState.TARGET_RESET

        # Use custom timing or fall back to controller's tdig_h
        if tdig_h_ns is not None:
            wait_time = Timer(tdig_h_ns, "ns")
        else:
            wait_time = self.tdig_h

        t_start = Timer(t_start_hold_ns, "ns") if t_start_hold_ns else wait_time
        t_stop = Timer(t_stop_hold_ns, "ns") if t_stop_hold_ns else wait_time

        self.log_info(f"Sending target reset pattern with {num_transitions} SDA transitions")

        # Start with SDA high, SCL low
        sda = 1
        self.sda = sda
        self.scl = 0
        await wait_time

        # Generate SDA transitions while SCL is low
        for _ in range(num_transitions):
            sda = 0 if sda else 1
            self.sda = sda
            await wait_time

        # Raise SCL and keep it high through START and STOP
        await wait_time
        self.scl = 1
        await wait_time

        # Send Repeated START (SDA falling while SCL high)
        self.sda = 0
        await t_start

        # Send STOP (SDA rising while SCL high)
        self.sda = 1
        await t_stop

        self.log_info("Target reset pattern complete")
        self.give_bus_control()

    async def send_target_reset_pattern_with_scl_glitch(
        self,
        glitch_at_transition: int = 7,
        tdig_h_ns: Optional[float] = None,
        glitch_on_sda_edge: bool = False,
    ) -> None:
        """
        Send target reset pattern with SCL glitch during SDA transitions.

        This should reset the transition counter and prevent reset detection.

        Args:
            glitch_at_transition: Which SDA transition to glitch SCL at (0-indexed)
            tdig_h_ns: Override timing in nanoseconds. If None, uses controller default.
        """
        await self.take_bus_control()
        self._state = I3cState.TARGET_RESET

        if tdig_h_ns is not None:
            wait_time = Timer(tdig_h_ns, "ns")
            half_wait = Timer(tdig_h_ns / 2, "ns")
        else:
            wait_time = self.tdig_h
            half_wait = Timer(self.timings.tdig_h / 2, "ns")

        self.log_info(f"Sending pattern with SCL glitch at transition {glitch_at_transition}")

        sda = 1
        self.sda = sda
        self.scl = 0
        await wait_time

        for i in range(14):
            sda = 0 if sda else 1
            self.sda = sda

            # Inject SCL glitch
            if i == glitch_at_transition:
                if not glitch_on_sda_edge:
                    await wait_time
                self.scl = 1
                await half_wait
                self.scl = 0
                await half_wait
            else:
                await wait_time

        # Complete pattern normally
        await wait_time
        self.scl = 1
        await wait_time

        self.sda = 0
        await wait_time

        self.sda = 1
        await wait_time

        self.give_bus_control()

    async def send_target_reset_pattern_with_sda_stable_low(
        self,
        tdig_h_ns: Optional[float] = None,
    ) -> None:
        """
        Send 14 SDA transitions, but then SDA goes stable low before SCL rises.

        This should cause FSM to return to AwaitPattern (abort).

        Args:
            tdig_h_ns: Override timing in nanoseconds. If None, uses controller default.
        """
        await self.take_bus_control()
        self._state = I3cState.TARGET_RESET

        if tdig_h_ns is not None:
            wait_time = Timer(tdig_h_ns, "ns")
        else:
            wait_time = self.tdig_h

        self.log_info("Sending pattern with SDA stable low during AwaitSCL")

        sda = 1
        self.sda = sda
        self.scl = 0
        await wait_time

        for _ in range(14):
            sda = 0 if sda else 1
            self.sda = sda
            await wait_time

        # SDA should be high after 14 toggles, but force it low
        self.sda = 0
        await wait_time
        await wait_time

        # Now raise SCL - but FSM should have aborted
        self.scl = 1
        await wait_time

        # Try to complete pattern anyway
        self.sda = 0
        await wait_time
        self.sda = 1
        await wait_time

        self.give_bus_control()

    async def send_target_reset_pattern_with_scl_drop_await_sr(
        self,
        tdig_h_ns: Optional[float] = None,
    ) -> None:
        """
        Send valid 14 transitions, SCL rises, but then drops before START.

        This tests the AwaitSr abort condition when SCL goes stable low.

        Args:
            tdig_h_ns: Override timing in nanoseconds. If None, uses controller default.
        """
        await self.take_bus_control()
        self._state = I3cState.TARGET_RESET

        if tdig_h_ns is not None:
            wait_time = Timer(tdig_h_ns, "ns")
            half_wait = Timer(tdig_h_ns / 2, "ns")
        else:
            wait_time = self.tdig_h
            half_wait = Timer(self.timings.tdig_h / 2, "ns")

        self.log_info("Sending pattern with SCL drop during AwaitSr")

        sda = 1
        self.sda = sda
        self.scl = 0
        await wait_time

        for _ in range(14):
            sda = 0 if sda else 1
            self.sda = sda
            await wait_time

        await wait_time
        self.scl = 1
        await half_wait

        # Drop SCL before START - this should abort AwaitSr
        self.scl = 0
        await wait_time

        # FSM should have aborted, attempt to complete anyway
        self.scl = 1
        await wait_time
        self.sda = 0
        await wait_time
        self.sda = 1
        await wait_time

        self.give_bus_control()

    async def send_target_reset_pattern_with_scl_drop_await_p(
        self,
        tdig_h_ns: Optional[float] = None,
        t_start_hold_ns: Optional[float] = None,
    ) -> None:
        """
        Complete valid pattern through START, but SCL drops before STOP.

        This tests the AwaitP abort condition when SCL goes stable low.
        NOTE: SCL must drop BEFORE SDA rises, because STOP is detected on SDA rising edge.

        Args:
            tdig_h_ns: Override timing in nanoseconds. If None, uses controller default.
            t_start_hold_ns: Override START hold time. If None, uses tdig_h_ns or default.
        """
        await self.take_bus_control()
        self._state = I3cState.TARGET_RESET

        if tdig_h_ns is not None:
            wait_time = Timer(tdig_h_ns, "ns")
        else:
            wait_time = self.tdig_h

        t_start = Timer(t_start_hold_ns, "ns") if t_start_hold_ns else wait_time

        self.log_info("Sending pattern with SCL drop during AwaitP")

        sda = 1
        self.sda = sda
        self.scl = 0
        await wait_time

        for _ in range(14):
            sda = 0 if sda else 1
            self.sda = sda
            await wait_time

        await wait_time
        self.scl = 1
        await wait_time

        # Valid START - SDA falls while SCL high
        self.sda = 0
        await t_start

        # Now in AwaitP, drop SCL BEFORE raising SDA
        self.scl = 0
        await wait_time

        # Now raise SDA (but SCL is low, so no STOP detected)
        self.sda = 1
        await wait_time

        # Return to idle
        self.scl = 1
        await wait_time

        self.give_bus_control()

    async def send_te0_error(self) -> None:
        """
        Trigger a TE0 error by sending START followed by 0x7E/R.

        Per I3C spec Sec.5.1.10.1.1, receipt of 7'h7E/R (broadcast address with
        read bit) after a dynamic address has been assigned is an Error Type
        TE0. The target enters HDR error mode (deaf) until it detects the HDR
        Exit Pattern or the optional 60µs timeout (Sec.5.1.10.1.9).

        After calling this method the controller still holds bus control.
        The caller must release the bus (e.g., send_hdr_exit + give_bus_control,
        or send_stop + give_bus_control) when done.
        """
        await self.take_bus_control()
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE, read=True)

    async def send_te1_error(self, ccc: int = 0x20) -> None:
        """
        Trigger a TE1 error by sending a CCC with bad T-bit parity.

        Per I3C spec Sec.5.1.10.1.2, if the target detects a parity error during
        a CCC code it cannot know whether the bus has changed to HDR mode.
        The target enters HDR error mode (deaf) until it detects the HDR Exit
        Pattern or the optional 60µs timeout (Sec.5.1.10.1.9).

        Args:
            ccc: CCC command code to send with bad parity (default: ENTHDR0 0x20).

        After calling this method the controller still holds bus control.
        The caller must release the bus when done.
        """
        await self.take_bus_control()
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_byte_tbit(ccc, inject_tbit_err=True)

    async def send_te2_error(
        self,
        ccc: int,
        defining_byte: int = None,
        target_addr: int = None,
        corrupt_defining_byte: bool = True,
    ) -> None:
        """
        Trigger a TE2 error by sending a CCC with bad T-bit parity on the
        defining byte or data byte.

        Per I3C spec Sec.5.1.10.1.3, a parity error on a CCC defining byte
        or data byte is a TE2 error.

        Args:
            ccc: CCC command code (sent with correct T-bit parity).
            defining_byte: If provided, sent after the CCC byte.
            corrupt_defining_byte: If True and defining_byte is provided,
                inject bad T-bit parity on the defining byte. If False,
                the defining byte gets correct parity (caller must send
                a data byte with bad parity separately).

        After calling this method the controller still holds bus control.
        The caller must send STOP and give_bus_control().
        """
        await self.take_bus_control()
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_byte_tbit(ccc, inject_tbit_err=False)
        if target_addr is not None:
            await self.write_addr_header(target_addr)
        if defining_byte is not None:
            await self.send_byte_tbit(defining_byte, inject_tbit_err=corrupt_defining_byte)

    async def set_bus_idle(self) -> None:
        """
        Set bus to idle state (both SDA and SCL high).

        Useful for initializing bus state before stress tests.
        """
        self.sda = 1
        self.scl = 1
        await self.tdig_h

    async def _run(self) -> None:
        """
        This coroutine is supposed to run in background and observe the bus state. It will not be
        enabled if there are no input SDA and SCL signals from the DUT. Once it detects the start
        pattern it processes the following IBI transfer.
        """
        while True:
            self.monitor_idle.set()
            if not self.monitor_enable.is_set():
                await self.monitor_enable.wait()
            self.monitor_idle.clear()

            # Wait for action on the bus
            next_state = await self.check_start()

            if next_state == I3cState.START:
                await self._handle_ibi()

            await NextTimeStep()
