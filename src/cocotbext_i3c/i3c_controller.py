"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

import logging
from enum import Enum
from typing import Any, Callable, Iterable, Optional, TypeVar, Union

import cocotb
from cocotb.handle import ModifiableObject
from cocotb.triggers import Event, FallingEdge, First, NextTimeStep, RisingEdge, Timer

from .common import (
    I3C_RSVD_BYTE,
    I3cControllerTimings,
    I3cState,
    I3cTargetResetAction,
    calculate_tbit,
    make_timer,
    report_config,
    with_timeout_event,
)

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

        self.tdig_h = make_timer(timings.tdig_h)
        self.thd = make_timer(timings.thd)
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
    ) -> None:
        """I3C Private Write transfer"""
        await self.take_bus_control()
        self.log_info(f"I3C: Write data ({mode.name}) {data} @ {hex(addr)}")
        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_start()
        await self.write_addr_header(addr)

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

    async def i3c_read(
        self, addr: int, count: int, stop: bool = True, mode: I3cXferMode = I3cXferMode.PRIVATE
    ) -> bytearray:
        """I3C Private Read transfer"""
        await self.take_bus_control()
        data = bytearray()
        self.log_info(f"I3C: Read data ({mode.name}) @ {hex(addr)}")

        await self.send_start()
        await self.write_addr_header(I3C_RSVD_BYTE)
        await self.send_start()
        await self.write_addr_header(addr, read=True)
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
        return data

    async def i3c_ccc_write(
        self,
        ccc: int,
        broadcast_data: Optional[Iterable[int]] = None,
        directed_data: Optional[Iterable[tuple[int, Iterable[int]]]] = None,
        defining_byte: Optional[int] = None,
        stop: bool = True,
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
