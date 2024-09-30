import logging
from enum import IntEnum
from typing import Any, Optional

import cocotb
from cocotb.handle import ModifiableObject
from cocotb.result import SimTimeoutError
from cocotb.triggers import (
    Edge,
    Event,
    FallingEdge,
    First,
    NextTimeStep,
    ReadOnly,
    RisingEdge,
    Timer,
    with_timeout,
)

from .common import (
    FULL_SPEED,
    I3C_RSVD_BYTE,
    I3cState,
    I3cTargetTimings,
    calculate_tbit,
    check_hold,
    check_in_time,
    report_config,
    with_timeout_event,
)


class I3cHeader(IntEnum):
    # No header has yet been issued; awaiting header after STOP
    NONE = 0
    # RESERVED header was intercepted
    RESERVED = 1
    # Request to read from this I3C Target was made
    READ = 2
    # Request to write to this I3C Target was made
    WRITE = 3
    # Address doesn't match the target or is unhandled CCC
    NON_APPLICABLE = 4


class I3CMemory:
    """
    Simple memory model that consists of an int array and read / write pointers.
    Since I3C has no memory address to direct the read / write requests to, the
    memory serves as a memory buffer.

    The `_read_mem` and `_write_mem` methods are mostly for the testing purposes -
    to read / write to the memory without the need to engage the target into private
    read / write operations.

    * TODO: Add the condition upon which the target should terminate / NACK
            the incoming requests. This can be treating the following memory model
            as a buffer and rejecting when empty / full.
    * TODO: Handle full / empty cases. Currently will overwrite the data.
    """

    def __init__(self, log: logging.Logger, size: int = 256) -> None:
        self.log = log
        self.size = size
        self.clear()

    def is_in_bound(self, address: int) -> None:
        assert address >= 0
        assert address <= self.size

    def _read_mem(self, address: int, length: int) -> list[int]:
        self.is_in_bound(address)
        data = self._mem[address : address + length]
        self.read_ptr = (self.read_ptr + length) % self.size
        return data

    def _write_mem(self, address: int, data: list[int], length: int) -> None:
        self.is_in_bound(address)
        for i in range(length):
            self._mem[address + i] = data[i]
        self.write_ptr = (self.write_ptr + length) % self.size

    def read(self, length: int = 1) -> list[int]:
        data = self._read_mem(self.read_ptr, length)
        self.log.debug(f"TARGET:::Performing read at {self.read_ptr}, data: {data}")
        return data

    def write(self, data: list[int], length: int = 1) -> None:
        self.log.debug(f"TARGET:::Performing write at {self.write_ptr}, data: {data}")
        self._write_mem(self.write_ptr, data, length)

    def clear(self):
        self._mem = [0 for _ in range(self.size)]
        self.read_ptr = 0
        self.write_ptr = 0

    def dump(self):
        for row in range(0, self.size, 20):
            cells = [f"0x{_:02X}" for _ in self._mem[row : min(self.size, row + 20)]]
            mem_row = ", ".join(cells)
            self.log.info(mem_row)


class I3CTarget:
    """
    Simple I3C Target with private reads / writes.

    This implementation should already:
        * Recognize the START condition & verify `BUS FREE` after STOP timing.
            * TODO: START recognition misses time-checking the `DS_OD` and `SU_OD` timings
        * Recognize and ACK the RESERVED BYTE header.
        * Intercept address header & decode it:
            * ACK if matches target's assigned address.
            * Ignore if address is CCC or other device's address.
              Will then await the `Sr` or `P` condition.
        * Distinguish private read from write and ack accordingly:
            * Should perform a read (in a loop for each data block):
                * Issue 8 bits of data from the memory currently pointed by the `read_ptr`
                * Follow it up by a transition bit (end of data):
                    * `1` by default (if there's data to be read)
                    * `0` otherwise, followed by await for `Sr` or `P` from the controller.
            * Should perform a write from the controller (in a loop, for each data block):
                * Will check first for `Sr` or `P`. Check is first in case there's `P` following
                  - both `P` and write data could be following either previous transition bit or
                    `Sr` condition.
                * Retrieve 8 bits of data.
                * Verify the odd-parity bit (fail if incorrect)
                * Save the byte into the memory.
            * Exit the flow upon `STOP` condition. Should then switch I3C Header to `None` and
              I3C State to `FREE`.

    * TODO: Handle NACKing incoming requests. Requires adding a condition upon which they
            are supposed to be rejected (e.g. `busy` for the purpose of testing).
    * TODO: Handle CCCs.
    * TODO: Investigate `Sr` detection after private write-read sequence with no `STOP` condition.
            At some point of the test sequence `Sr` is not correctly intercepted after `DATA_RD`
            state. See also: FIXME in the test_recovery.
    * TODO: Add error recovery - illegal sequence will cause a simulation fail.
    """

    def __init__(
        self,
        sda_i: ModifiableObject,
        sda_o: ModifiableObject,
        scl_i: ModifiableObject,
        scl_o: ModifiableObject,
        debug_state_o: Optional[ModifiableObject] = None,
        debug_detected_header_o: Optional[ModifiableObject] = None,
        timings: Optional[I3cTargetTimings] = None,
        speed: float = FULL_SPEED,
        address: int = None,  # TODO: Should be assigned with dynamic address assignment
        max_read_length: int = 2,  # TODO: Should be controlled with CCC
        *args,
        **kwargs,
    ) -> None:
        self.log = logging.getLogger(f"cocotb.{sda_o._path}")
        self.log.setLevel("INFO")
        self.sda_i = sda_i
        self.sda_o = sda_o
        self.scl_i = scl_i
        self.scl_o = scl_o
        self.debug_state_o = debug_state_o
        self.debug_detected_header_o = debug_detected_header_o
        self.speed = speed
        self.address = address
        self.max_read_length = max_read_length

        if timings is None:
            timings = I3cTargetTimings()
        self.timings = timings

        if address is not None:
            self.log.info(f"TARGET:::Using static address for I3C Target: {hex(address)}")

        # Initialize memory
        self._mem = I3CMemory(self.log)
        super().__init__(*args, **kwargs)

        if self.sda_o is not None:
            self.sda_o.setimmediatevalue(1)

        if self.scl_o is not None:
            self.scl_o.setimmediatevalue(1)

        self._state_ = I3cState.FREE
        self.state = I3cState.FREE

        self._header = I3cHeader.NONE
        report_config(self.speed, timings, self.log.info)

        self.monitor_enable = Event()
        self.monitor_enable.set()
        self.monitor_idle = Event()
        cocotb.start_soon(self._run())

        self.hdr_exit_detected = False
        cocotb.start_soon(self._detect_hdr_exit())

    @property
    def bus_active(self) -> bool:
        return self.state is not I3cState.FREE

    @property
    def state(self) -> I3cState:
        return self._state_

    @state.setter
    def state(self, value: I3cState) -> None:
        self._state_ = value
        if self.debug_state_o is not None:
            self.debug_state_o.setimmediatevalue(value)

    @property
    def scl(self) -> Any:
        return self.scl_i.value

    @property
    def sda(self) -> Any:
        return self.sda_i.value

    @sda.setter
    def sda(self, value: Any) -> None:
        self.sda_o.value = value

    @property
    def header(self) -> I3cHeader:
        return self._header

    @header.setter
    def header(self, value: I3cHeader):
        self._header = value
        if self.debug_detected_header_o:
            self.debug_detected_header_o.setimmediatevalue(value)

    async def check_start(self, repeated=True):
        if not (self.sda and self.scl):
            return None

        if repeated:
            assert self.bus_active
            tCAS = self.timings.tcasr
            next_state = I3cState.RS
        else:
            assert not self.bus_active
            tCAS = self.timings.tcas
            next_state = I3cState.START

        # Check if the condition for FREE bus is satisfied (applies to START only)
        if not repeated:
            try:
                await check_hold([self.sda_i, self.scl_i], tCAS, "ns")
            except SimTimeoutError as e:
                self.log.debug(e)
                return None

        # Check clock before Repeated START
        if repeated:
            try:
                await check_hold([self.sda_i, self.scl_i], self.timings.tcbsr, "ns")
            except SimTimeoutError as e:
                self.log.debug(e)
                return None

        sda_falling_edge = FallingEdge(self.sda_i)
        scl_falling_edge = FallingEdge(self.scl_i)
        result = None
        monitor_enable = self.monitor_enable.is_set()
        while ((not monitor_enable) or (monitor_enable and self.monitor_enable.is_set())) and result is None:
            try:
                result = await with_timeout(First(sda_falling_edge, scl_falling_edge), 1, "ns")
            except SimTimeoutError:
                self.log.debug("Waiting for SDA/SCL falling edge")

        if result != sda_falling_edge:
            return None
        try:
            await check_in_time(FallingEdge(self.scl_i), tCAS)
        except Exception:
            self.log.error("SCL did not fall in time")
            return None

        # TODO: Add timing check, as the `sda` should be raised no earlier than `ds_od`
        # Followed by `scl` being raised no earlier than `tsu_od`
        if not repeated:
            sda_rising_edge = RisingEdge(self.sda_i)
            scl_rising_edge = RisingEdge(self.scl_i)
            result = await First(sda_rising_edge, scl_rising_edge)
            if result != sda_rising_edge:
                return None

        self.state = next_state
        return next_state

    async def check_stop(self):
        await RisingEdge(self.scl_i)
        if self.sda:
            return None

        rising_sda = RisingEdge(self.sda_i)
        falling_scl = FallingEdge(self.scl_i)

        try:
            self.log.debug("Wait for rising_sda or falling_scl")
            first_rising_edge, _ = await check_in_time(First(rising_sda, falling_scl), self.timings.tcbp)
        except Exception:
            return None

        # The `and self.scl` is necessary in case both edges occur at the same time
        # `First` then chooses one of the options to return arbitrarily
        if first_rising_edge != rising_sda or self.scl_i.value == 0:
            return None

        self.state = I3cState.STOP
        return I3cState.STOP

    async def check_start_or_stop(self):
        """
        Detect repeated START (Sr) or STOP (P) condition for read / write messages.
        """
        self.state = I3cState.AWAIT_SR_OR_P
        state = None
        assert self.bus_active
        await ReadOnly()

        if self.sda and self.scl:
            return await self.check_start(repeated=True)

        if not self.sda and not self.scl:
            return await self.check_stop()

        if not self.scl and self.sda:
            state = await self.check_stop()

        if self.scl and not self.sda and not state:
            state = await self.check_start(repeated=True)
        return state

    async def recv_bit(self) -> bool:
        assert self.bus_active
        # Sample data on the rising clock edge
        if not self.scl:
            await RisingEdge(self.scl_i)
        bit = bool(self.sda)
        await FallingEdge(self.scl_i)
        return bit

    async def verify_parity(self, byte) -> bool:
        self.state = I3cState.TBIT_WR
        expected_parity_bit = int(calculate_tbit(byte))

        await RisingEdge(self.scl_i)

        parity_bit = bool(self.sda)

        assert expected_parity_bit == parity_bit, (
            f"Received transition bit: {parity_bit} doesn't match given data: {hex(byte)}. "
            f"Expected {expected_parity_bit}."
        )
        await FallingEdge(self.scl_i)

    async def ack(self):
        self.state = I3cState.ACK
        if self.scl:
            await FallingEdge(self.scl_i)
        self.sda = 0
        await FallingEdge(self.scl_i)
        self.sda = 1

    async def recv(self, bits_num=8) -> int:
        b = 0
        for _ in range(bits_num):
            b = (b << 1) | await self.recv_bit()
        return b

    async def recv_byte(self, is_data: bool = True, ack=True, check_for_stop=False) -> int:
        length = 8
        s, b = 0, 0
        next_state = None
        if check_for_stop:
            next_state = await self.check_stop()
            if next_state == I3cState.STOP:
                return 0xFF, next_state
            if not self.scl and not self.sda:
                s = 1
                b = bool(self.sda)
        self.state = I3cState.DATA_WR
        for _ in range(s, length):
            b = (b << 1) | await self.recv_bit()

        if is_data:
            await self.verify_parity(b)
        elif ack:
            await self.ack()

        next_state = await self.check_start(repeated=True)
        return b, next_state

    async def send_bit(self, bit: bool):
        if self.scl:
            await FallingEdge(self.scl_i)

        self.sda = bool(bit)

        await FallingEdge(self.scl_i)
        # TODO: Ensure that the sent bit was propagated on the bus
        # assert self.sda_i.value == int(bit)
        self.sda = 1

    async def send_byte(self, byte: int, terminate: bool):
        for i in range(8):
            await self.send_bit(byte & (1 << 7 - i))

        self.state = I3cState.TBIT_RD
        if self.scl:
            await FallingEdge(self.scl_i)

        # Issue end of data if there's no more data to be send
        self.sda = not terminate
        await RisingEdge(self.scl_i)
        self.sda = 1

        # Wait for Sr or P if termination requested
        next_state = None
        if terminate:
            if await self.check_stop():
                next_state = I3cState.STOP
            elif await self.check_start(repeated=True):
                next_state = I3cState.RS

        return next_state

    async def wait_header(self) -> None:
        self.state = I3cState.ADDR
        addr_header = await self.recv(bits_num=8)
        addr, is_read = addr_header >> 1, addr_header & 0x1

        self.log.warning(f"TARGET:::Address: {hex(addr)} RnW: {is_read}")
        self.log.warning(f"My address: {hex(self.address)}")

        if addr == I3C_RSVD_BYTE:
            assert self.header in [I3cHeader.NONE, I3cHeader.READ, I3cHeader.WRITE]
            await self.ack()
            self.header = I3cHeader.RESERVED
        elif addr == self.address:
            assert self.header in [I3cHeader.RESERVED, I3cHeader.READ, I3cHeader.WRITE]
            await self.ack()
            self.header = I3cHeader.READ if is_read else I3cHeader.WRITE
        else:
            self.header = I3cHeader.NON_APPLICABLE

    async def handle_read(self) -> int:
        """I3C Private Read Transfer"""
        next_state = None
        while not next_state:
            self.state = I3cState.DATA_RD
            data = self._mem.read()
            tbit = self._mem.read_ptr < self._mem.write_ptr
            next_state = await self.send_byte(data[0] & 0xFF, not tbit)
        self.state = next_state
        return next_state

    async def handle_write(self) -> None:
        """I3C Private Write Transfer"""
        next_state = None
        while not next_state:
            self.state = I3cState.DATA_WR
            data, next_state = await self.recv_byte(is_data=True, ack=False, check_for_stop=True)
            if next_state != I3cState.STOP:
                self._mem.write([data & 0xFF])
        self.state = next_state
        return next_state

    async def handle_message(self):
        await self.wait_header()
        match self.header:
            case I3cHeader.RESERVED:
                self.state = I3cState.AWAIT_SR_OR_P
                next_state = None
                while not next_state:
                    next_state = await self.check_start_or_stop()
            case I3cHeader.READ:
                next_state = await self.handle_read()
            case I3cHeader.WRITE:
                next_state = await self.handle_write()
            case I3cHeader.NON_APPLICABLE:
                next_state = None
                while not next_state:
                    next_state = await self.check_start_or_stop()
                self.header = I3cHeader.NONE
            case _:
                raise Exception(
                    f"Intercepted address header: {self.header}"
                    "Expected one of:"[
                        I3cHeader.RESERVED,
                        I3cHeader.READ,
                        I3cHeader.WRITE,
                        I3cHeader.NON_APPLICABLE,
                    ]
                )
        return next_state

    def clear_hdr_exit_flag(self):
        self.hdr_exit_detected = False

    async def _detect_hdr_exit(self):
        self.log.info("Starting HDR Exit Pattern detection monitor")
        while True:
            if not (not self.scl and self.sda):
                await NextTimeStep()
                continue

            for _ in range(4):
                rising_scl = RisingEdge(self.scl_i)
                falling_sda = FallingEdge(self.sda_i)
                trigger = await First(rising_scl, falling_sda)
                if trigger == rising_scl:
                    break

            if trigger == rising_scl:
                continue

            self.hdr_exit_detected = True
            self.log.info("Detected HDR Exit Pattern!")

    async def send_ibi(self, mdb=None, data: bytearray = None):
        # Disable bus monitor and wait for bus to enter idle state
        self.monitor_enable.clear()
        if not self.monitor_idle.is_set():
            await self.monitor_idle.wait()
        assert not self.bus_active

        # Issue START condition
        self.sda = 0

        # For now expect IBI accept but the controller can also NACK
        await FallingEdge(self.scl_i)

        # Send address with RnW bit set to 1'b1
        terminate = mdb is None and data is None
        next_state = await self.send_byte((self.address << 1) | 1, terminate=terminate)
        if mdb:
            next_state = await self.send_byte(mdb, terminate=bool(not data))

        while data:
            value = data.pop(0)
            terminate = not bool(len(data))
            next_state = await self.send_byte(value, terminate=terminate)

        self.state = next_state
        if self.state == I3cState.STOP:
            self.log.debug("TARGET:::Got STOP.")
            self.state = I3cState.FREE
            self.header = I3cHeader.NONE

        # Finish IBI handling and re-enable bus monitor
        self.monitor_enable.set()

    async def _run(self) -> None:
        while True:
            # Monitor is idle, it will check whether it is enabled before observing the bus
            self.monitor_idle.set()
            if not self.monitor_enable.is_set():
                self.log.debug("Monitor disabled, awaiting for external enable trigger")
                await self.monitor_enable.wait()
            self.monitor_enable.set()

            # From this moment monitor is busy and should not be interrupted
            self.monitor_idle.clear()

            # Wait for action on the bus
            next_state = await self.check_start(repeated=False)
            if next_state:
                self.state = next_state
            else:
                await with_timeout_event(
                    self.monitor_enable,
                    First(Edge(self.sda_i), Edge(self.scl_i)),
                    1,
                )

            while self.bus_active:
                self.state = await self.handle_message()

                if self.state == I3cState.STOP:
                    self.log.debug("TARGET:::Got STOP.")
                    self.state = I3cState.FREE
                    self.header = I3cHeader.NONE
