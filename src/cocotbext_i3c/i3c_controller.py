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

        self.thd = make_timer(timings.thd)
        self.tdig_h = make_timer(timings.tdig_h)
        self.tdig_h_minus_thd = make_timer(at_least_tsupp(timings.tdig_h - timings.thd))
        self.tdig_l = make_timer(at_least_tsupp(timings.tdig_l))
        self.tdig_l_minus_thd = make_timer(at_least_tsupp(timings.tdig_l - timings.thd))
        self.tsu_od = make_timer(timings.tsu_od)
        self.tcas = make_timer(timings.tcas)
        self.tcbp = make_timer(timings.tcbp)
        self.tcbsr = make_timer(timings.tcbsr)
        self.tcbsr_half = make_timer(timings.tcbsr / 2)
        self.tcasr = make_timer(timings.tcasr)
        self.tfree = make_timer(timings.tfree)
        self.tsco = make_timer(timings.tsco)
        self.tsu_pp = make_timer(timings.tsu_od)

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

    async def send_byte(self, b: int, addr: bool = False) -> bool:
        self._state = I3cState.ADDR if addr else I3cState.DATA_WR
        for i in range(8):
            await self.send_bit(b & (1 << 7 - i))
        self._state = I3cState.ACK
        return await self.recv_bit_od()

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
        if addr == I3C_RSVD_BYTE:
            self.log_info("Address Header:::Reserved I3C Address Header 0x%02x", addr)
        else:
            self.log_info("Address Header:::Address Header to device at I3C address 0x%02x", addr)
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
            (byte, tgt_stop) = await self.recv_byte_t_bit(stop=stop and not length)
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
    ) -> I3cPWResp:
        """I3C Private Write transfer"""
        await self.take_bus_control()
        self.log_info(f"I3C: Write data ({mode.name}) {data} @ {hex(addr)}")
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_start()
        ack = await self.write_addr_header(addr)
        if ack:
            for i, d in enumerate(data):
                match mode:
                    case I3cXferMode.PRIVATE:
                        await self.send_byte_tbit(d, inject_tbit_err)
                    case I3cXferMode.LEGACY_I2C:
                        await self.send_byte(d)
                self.log_info(f"I3C: wrote byte {hex(d)}, idx={i}")

        if stop:
            await self.send_stop()

        self.give_bus_control()
        return I3cPWResp(not ack, len(data))

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

    async def _handle_ibi(self):
        """
        Receive and IBI from the target, support for MDB is determined from the `self.targets` list
        which should be configured by the testbench. If there is no entry for the target with an address
        received on the bus, assume that the BCR has no set values hence is equal to 0.
        """

        # Accept/reject the interrupt by sending an ACK/NACK
        ack = not self.nack_ibis.is_set()
        addr = await self.recv_byte(send_ack=ack) >> 1

        # Receive IBI
        data = bytearray()
        if ack:
            self.log.info(f"ACK-ed an IBI from 0x{addr:02X}")
            target_idx = self.get_target_idx_by_addr(addr)
            if target_idx is not None:
                self.targets[target_idx].set_bcr_fields()
                target = self.targets[target_idx]
                mdb_enabled = target.bcr & (1 << 2)
                if mdb_enabled:
                    await self.recv_until_eod_tbit(data, self.max_ibi_data_len + 1)
                    self.log.info(
                        f"IBI MDB: 0x{data[0]:02X}, data: ["
                        + " ".join([f"0x{d:02X}" for d in data[1:]])
                        + "]"
                    )
            else:
                self.log.warning(f"Target ({hex(addr)}) has no configured BCR, assuming BCR = 0")
        else:
            self.log.info(f"NACK-ed an IBI from 0x{addr:02X}")

        # Send stop
        await self.send_stop()

        if ack:
            self.got_ibi.set(bytearray([addr]) + data)

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
