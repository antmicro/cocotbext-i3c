"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

from dataclasses import dataclass
import logging
from enum import Enum, IntEnum
from typing import Any, Iterable, Optional
from cocotb.triggers import Timer
from cocotb.handle import ModifiableObject

class I3cState(IntEnum):
    FREE = 0
    START = 1
    ADDR_SEND = 2
    DATA_SEND = 3
    DATA_RECV = 4
    ACK = 5
    RS = 6
    TBIT_WR = 7
    TBIT_RD = 8
    CCC = 9
    STOP = 10

@dataclass
class I3cTimings:
    # Minimum timings, push-pull mode (ns)
    thigh_d: float = 32.0      # High Period of SCL Clock (for Pure Bus)
    tlow_d: float = 32.0       # SCL Clock Low Period
    tcas: float = 38.4         # Clock After START (S) Condition
    tcbp: float = 19.2         # Clock Before STOP (P) Condition
    tcbsr: float = 19.2        # Clock Before Repeated START (Sr) Condition
    tcasr: float = 19.2        # Clock After Repeated START (Sr) Condition
    # NOTE: tlow_od is excluded because this timing is there only to give the pull-up
    # resitor enough time to pull the SDA line up - this is relevant only in case of
    # real hardware or analog circuit simulations.
    #tlow_od: float = 200.0     # Low Period of SCL Clock (Open Drain)
    # TODO: This timing is used to allow I3C devices to disable I2C Spike filter
    # and should be used for an initial broadcast
    #thigh_init: float = 200.0  # High Period of SCL Clock (for First Broadcast Address)
    tfree: float = 38.4        # Bus Free condition (for Pure bus)
    # Bus Available and Bus Idle conditions are of no concern for this model
    # they matter only for target-issued xfers and hot-join events.
    tsu_od: float = 3.0        # Open-drain set-up time
    tsupp: float = 3.0         # SDA Set-up time
    thd: float = 6.0           # SDA Hold time
    tsco: float = 12.0         # Clock in to Data Out for Target (max)

class I3cXferMode(Enum):
    PRIVATE = 0
    LEGACY_I2C = 1

    @property
    def name(self) -> str:
        match self:
            case I3cXferMode.PRIVATE: return "Private"
            case I3cXferMode.LEGACY_I2C: return "Legacy I2C"

class I3cController:
    RSVD_BYTE: int = 0x7e
    FULL_SPEED: float = 12.5e6

    def __init__(
        self,
        sda_i: ModifiableObject,
        sda_o: ModifiableObject,
        scl_i: ModifiableObject,
        scl_o: ModifiableObject,
        debug_state_o: Optional[ModifiableObject] = None,
        timings: Optional[I3cTimings] = None,
        speed: float = FULL_SPEED,
        silent=True,
        *args,
        **kwargs,
    ) -> None:
        self.log = logging.getLogger(f"cocotb.{sda_o._path}")
        self.sda_i = sda_i
        self.sda_o = sda_o
        self.scl_i = scl_i
        self.scl_o = scl_o
        self.debug_state_o = debug_state_o
        self.speed = speed

        self.silent = silent

        if timings is None:
            timings = I3cTimings()

        def scaled_timing(period_ns: float) -> float:
            return (12.5e6 / speed) * period_ns

        def make_timer(period_ns: float) -> Timer:
            return Timer(scaled_timing(period_ns), "ns")

        def at_least_tsupp(period_ns: float) -> float:
            return period_ns if period_ns > timings.tsupp else timings.tsupp

        self.thigh_d = make_timer(timings.thigh_d)
        self.thd = make_timer(timings.thd)
        self.tlow_d = make_timer(at_least_tsupp(timings.tlow_d))
        self.tlow_d_minus_thd = make_timer(at_least_tsupp(timings.tlow_d - timings.thd))
        self.tlow_d_minus_tsco = make_timer(at_least_tsupp(timings.tlow_d - timings.tsco))
        self.tsu_od = make_timer(timings.tsu_od)
        self.tcas = make_timer(timings.tcas)
        self.tcbp = make_timer(timings.tcbp)
        self.tcbsr = make_timer(timings.tcbsr)
        self.tcbsr_half = make_timer(timings.tcbsr / 2)
        self.tcasr = make_timer(timings.tcasr)
        self.tfree = make_timer(timings.tfree)
        self.tsco = make_timer(timings.tsco)

        self.hold_data = False

        super().__init__(*args, **kwargs)

        if self.sda_o is not None:
            self.sda_o.setimmediatevalue(1)
        if self.scl_o is not None:
            self.scl_o.setimmediatevalue(1)
        self._state_ = I3cState.FREE
        self._state = I3cState.FREE

        self.log_info("I3C Controller configuration:")
        self.log_info(f"  Rate: {self.speed / 1000.0}kHz "
                    f"({100.0 * self.speed / I3cController.FULL_SPEED}%)")
        self.log_info( "  Timings:")
        self.log_info(f"    SCL Clock High Period: {scaled_timing(timings.thigh_d)}ns")
        self.log_info(f"    SCL Clock Low Period: {scaled_timing(timings.tlow_d)}ns")
        self.log_info(f"    Clock After START (S) Condition: {scaled_timing(timings.tcas)}ns")
        self.log_info(f"    Clock Before STOP (P) Condition: {scaled_timing(timings.tcbp)}ns")
        self.log_info( "    Clock Before Repeated START (Sr) Condition: "
                      f"{scaled_timing(timings.tcbsr)}ns")
        self.log_info( "    Clock After Repeated START (Sr) Condition: "
                      f"{scaled_timing(timings.tcasr)}ns")
        self.log_info(f"    Bus Free condition: {scaled_timing(timings.tfree)}ns")
        self.log_info(f"    Open-drain set-up time: {scaled_timing(timings.tsu_od)}ns")
        self.log_info(f"    SDA Set-up time (Push-Pull): {scaled_timing(timings.tsupp)}ns")
        self.log_info(f"    SDA Hold time (Push-Pull): {scaled_timing(timings.thd)}ns")
        self.log_info(f"    Clock in to Data Out for Target: {scaled_timing(timings.tsco)}ns")

    def log_info(self, *args):
        if self.silent: return
        self.log.info(*args)

    @property
    def bus_active(self) -> bool:
        return self._state is not I3cState.FREE

    @property
    def _state(self) -> I3cState:
        return self._state_

    @property
    def remaining_tlow(self) -> Timer:
        return self.tlow_d if not self.hold_data else self.tlow_d_minus_thd

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

    async def send_start(self) -> None:
        if self.bus_active:
            self._state = I3cState.RS
            self.scl = 0
            await self.thd
            self.sda = 1
            await self.tlow_d_minus_thd
        else:
            self._state = I3cState.START

        self.sda = 1
        self.scl = 1
        # Open-drain address to Push-Pull data handoff procedure description
        # says the time of SCL held high can be as low as T_{DIG_H}, however
        # min(T_{CBSr}) + min(T{CASr}) > min(T_{DIG_H})
        await self.tcbsr

        self.sda = 0
        await (self.tcas if not self.bus_active else self.tcasr)
        self.scl = 0
        await self.tcasr

        self.hold_data = False

    async def send_stop(self) -> None:
        self.log_info("I3C: STOP")
        self._state = I3cState.STOP
        if not self.bus_active:
            return

        self.scl = 0
        if self.hold_data: await self.thd
        self.sda = 0
        await self.remaining_tlow
        self.scl = 1
        await self.tcbp
        self.sda = 1
        await self.tfree

        self._state = I3cState.FREE
        self.hold_data = False

    async def send_bit(self, b: bool) -> None:
        if not self.bus_active:
            self.send_start()

        self.scl = 0
        if self.hold_data: await self.thd
        self.sda = bool(b)
        await self.remaining_tlow
        self.scl = 1
        await self.thigh_d
        self.hold_data = True

    async def recv_bit(self) -> bool:
        if not self.bus_active:
            self.send_start()

        self.scl = 0
        if self.hold_data: await self.thd
        self.sda = 1
        await self.remaining_tlow
        if self.sda_i is None:
            b = False
        else:
            b = bool(self.sda)
        self.scl = 1
        await self.thigh_d
        self.hold_data = False

        return b

    async def recv_bit_od(self) -> bool:
        if not self.bus_active:
            self.send_start()

        self.scl = 0
        # We don't hold the data here, because it's on the target to pull it down
        # after the required amount of time
        await self.tlow_d
        if self.sda_i is None:
            b = False
        else:
            b = bool(self.sda)
        self.scl = 1
        await self.thigh_d
        self.hold_data = False

        return b

    async def send_byte(self, b: int, addr: bool = False) -> bool:
        self._state = I3cState.ADDR_SEND if addr else I3cState.DATA_SEND
        for i in range(8):
            await self.send_bit(b & (1 << 7 - i))
        self._state = I3cState.ACK
        return await self.recv_bit_od()

    async def recv_byte(self, ack: bool) -> int:
        b = 0
        self._state = I3cState.DATA_RECV
        for _ in range(8):
            b = (b << 1) | await self.recv_bit()
        self._state = I3cState.ACK
        await self.send_bit(ack)
        return b

    @staticmethod
    def calculate_tbit(value: int) -> bool:
        tbit = True
        while value != 0:
            if (value & 1) != 0:
                tbit = not tbit
            value >>= 1
        return tbit

    async def send_byte_tbit(self, b: int) -> None:
        self._state = I3cState.DATA_SEND
        for i in range(8):
            await self.send_bit(bool(b & (1 << (7 - i))))
        # Send T-Bit
        self._state = I3cState.TBIT_WR
        await self.send_bit(I3cController.calculate_tbit(b))

    async def tbit_eod(self, request_end: bool) -> bool:
        self.scl = 0
        await self.tsco
        eod = not bool(self.sda)
        await self.tlow_d_minus_tsco
        # At this point target should set SDA to High-Z.
        self.scl = 1
        if eod: # Target requests end-of-data
            self.sda = 0
            self.hold_data = False
            await self.thigh_d
            # This should be followed by a stop signal: self.send_stop
        elif request_end: # Controller requests end-of-data
            # This is basically RS and should be followed by a stop signal: self.send_stop
            await self.tcbsr
            self.sda = 0
            await self.tcasr
        else:
            self.hold_data = False
            await self.thigh_d

        return eod


    async def recv_byte_t_bit(self, stop: bool) -> tuple[int, bool]:
        b = 0
        self._state = I3cState.DATA_RECV
        for _ in range(8):
            b = (b << 1) | (1 if await self.recv_bit() else 0)
        self._state = I3cState.TBIT_RD
        tgt_eod = await self.tbit_eod(request_end=stop)
        return (b, tgt_eod | stop)

    async def write_addr_header(self, addr: int, read: bool = False) -> None:
        if addr == I3cController.RSVD_BYTE:
            self.log_info("Address Header:::Reserved I3C Address Header 0x%02x", addr)
        else:
            self.log_info("Address Header:::Address Header to device at I3C address 0x%02x", addr)
        nack = await self.send_byte((addr << 1) | (0 if not read else 1), addr=True)
        if nack:
            self.log_info("Address Header:::Got NACK")
        else:
            self.log_info("Address Header:::Got ACK")

    async def recv_until_eod_tbit(self, buf: bytearray, count: int) -> None:
        for i in range(count):
            (byte, stop) = await self.recv_byte_t_bit(stop = (i == count - 1))
            self.log_info(f"I3C: read byte {hex(byte)}, idx={i}")
            buf.append(byte)
            if stop: return

    async def i3c_write(self, addr: int, data: Iterable[int], stop: bool = True,
                        mode: I3cXferMode = I3cXferMode.PRIVATE) -> None:
        """ I3C Private Write transfer """
        self.log_info(f"I3C: Write data ({mode.name}) {data} @ {hex(addr)}")
        await self.send_start()
        await self.write_addr_header(I3cController.RSVD_BYTE)
        await self.send_start()
        await self.write_addr_header(addr)

        for i, d in enumerate(data):
            match mode:
                case I3cXferMode.PRIVATE: await self.send_byte_tbit(d)
                case I3cXferMode.LEGACY_I2C: await self.send_byte(d)
            self.log_info(f"I3C: wrote byte {hex(d)}, idx={i}")

        if stop: await self.send_stop()

    async def i3c_read(self, addr: int, count: int, stop: bool = True,
                       mode: I3cXferMode = I3cXferMode.PRIVATE) -> bytearray:
        """ I3C Private Read transfer """
        data = bytearray()
        self.log_info(f"I3C: Read data ({mode.name}) @ {hex(addr)}")

        await self.send_start()
        await self.write_addr_header(I3cController.RSVD_BYTE)
        await self.send_start()
        await self.write_addr_header(addr, read=True)
        match mode:
            case I3cXferMode.PRIVATE: await self.recv_until_eod_tbit(data, count)
            case I3cXferMode.LEGACY_I2C:
                for i in range(count):
                    data.append(await self.recv_byte(i == count - 1))
        if stop: await self.send_stop()

        return data

    async def i3c_ccc_write(self, ccc: int,
                            broadcast_data: Optional[Iterable[int]] = None,
                            directed_data: Optional[Iterable[tuple[int, Iterable[int]]]] = None,
                            defining_byte: Optional[int] = None,
                            stop: bool = True) -> None:
        """ Issue CCC Write frame. For directed CCCs use an iterable of address-data tuples"""
        is_broadcast = ccc <= 0x7f

        log_data = broadcast_data if is_broadcast else directed_data
        if is_broadcast:
            self.log_info(f"I3C: CCC {hex(ccc)} WR (Broadcast): {log_data}")
        else:
            self.log_info(f"I3C: CCC {hex(ccc)} WR (Directed): {log_data}")

        await self.send_start()
        await self.write_addr_header(I3cController.RSVD_BYTE)
        await self.send_byte_tbit(ccc)
        if defining_byte is not None:
            await self.send_byte_tbit(defining_byte)

        if is_broadcast:
            assert broadcast_data is not None

            for byte in broadcast_data:
                await self.send_byte_tbit(byte)
        else:
            assert directed_data is not None

            for addr, data in directed_data:
                await self.send_start()
                await self.write_addr_header(addr)
                for byte in data:
                    await self.send_byte_tbit(byte)

        if stop:
            await self.send_stop()

    async def i3c_ccc_read(self, ccc: int, addr: int, count: int,
                           defining_byte: Optional[int] = None, stop: bool = True) -> bytearray:
        """ Issue CCC Read frame. For directed CCCs use an iterable of address-data tuples"""
        data = bytearray()
        self.log_info(f"I3C: CCC {hex(ccc)} RD (Directed @ {hex(addr)})")

        await self.send_start()
        await self.write_addr_header(I3cController.RSVD_BYTE)
        await self.send_byte_tbit(ccc)
        if defining_byte is not None:
            await self.send_byte_tbit(defining_byte)
        await self.send_start()
        await self.write_addr_header(addr, read=True)
        await self.recv_until_eod_tbit(data, count)

        if stop:
            await self.send_stop()

        return data
