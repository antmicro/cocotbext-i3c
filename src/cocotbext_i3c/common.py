#!/usr/bin/env python3
import logging
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable

from cocotb.result import SimTimeoutError
from cocotb.triggers import NextTimeStep, Timer, with_timeout
from cocotb.utils import _get_log_time_scale, _get_simulator_precision, get_sim_time

I3C_RSVD_BYTE: int = 0x7E
FULL_SPEED: float = 12.5e6


class I3cState(IntEnum):
    FREE = 0
    START = 1
    ADDR = 2
    DATA_WR = 3
    DATA_RD = 4
    ACK = 5
    RS = 6
    TBIT_WR = 7
    TBIT_RD = 8
    CCC = 9
    STOP = 10
    AWAIT_SR_OR_P = 11
    TARGET_RESET = 12


@dataclass
class I3cTimings:
    # Minimum timings, push-pull mode (ns)
    # Based on:
    # * Table 86 'I3C Open Drain Timing Parameter'
    # * Table 87 'I3C Push-Pull Timing Parameters for SDR, ML, HDR-DDR, and HDR-BT Modes'
    # of I3C Basic Specification v1.1.1
    thd: float  # SDA Hold time
    tdig_h: float = 32.0  # SCL Clock High Period (for Pure Bus)
    tdig_l: float = 32.0  # SCL Clock Low Period
    tcas: float = 38.4  # Clock After START (S) Condition
    tcbp: float = 19.2  # Clock Before STOP (P) Condition
    tcbsr: float = 19.2  # Clock Before Repeated START (Sr) Condition
    tcasr: float = 19.2  # Clock After Repeated START (Sr) Condition
    # NOTE: tlow_od is excluded because this timing is there only to give the pull-up
    # resistor enough time to pull the SDA line up - this is relevant only in case of
    # real hardware or analog circuit simulations.
    # tlow_od: float = 200.0     # Low Period of SCL Clock (Open Drain)
    # TODO: This timing is used to allow I3C devices to disable I2C Spike filter
    # and should be used for an initial broadcast
    # thigh_init: float = 200.0  # High Period of SCL Clock (for First Broadcast Address)
    tfree: float = 38.4  # Bus Free condition (for Pure bus)
    # Bus Available and Bus Idle conditions are of no concern for this model
    # they matter only for target-issued xfers and hot-join events.
    tsu_od: float = 3.0  # Open-drain set-up time
    tsupp: float = 3.0  # SDA Set-up time
    tsco: float = 12.0  # Clock in to Data Out for Target (max)


@dataclass
class I3cControllerTimings(I3cTimings):
    thd: float = 6.0


@dataclass
class I3cTargetTimings(I3cTimings):
    thd: float = 0.0


def scaled_timing(period_ns: float, speed: float) -> float:
    return (12.5e6 / speed) * period_ns


def make_timer(period_ns: float, speed: float = FULL_SPEED) -> Timer:
    return Timer(scaled_timing(period_ns, speed), "ns")


def calculate_tbit(value: int, inject_tbit_err: bool = False) -> bool:
    """Calculates odd-parity for `value` to be written by the controller after `value`."""
    tbit = True
    while value != 0:
        if (value & 1) != 0:
            tbit = not tbit
        value >>= 1
    return not tbit if inject_tbit_err else tbit


def round_time_to_sim_precision(time, units="ns"):
    """Rounds up measured time to simulator precision for hold time checks."""
    return round(time, abs(_get_simulator_precision() - _get_log_time_scale(units)))


async def check_in_time(trigger, time_in, units="ns"):
    """
    Checks if `trigger` occurred no earlier than `time_in` `units`.
    """
    start = get_sim_time(units)
    result = await trigger
    end = get_sim_time(units)

    total_time = round_time_to_sim_precision(end - start)

    if total_time < time_in:
        logging.error(f"TIMEOUT: expected: {time_in} got {total_time}")
    assert total_time >= time_in, (
        f"Elapsed time for {trigger}: {total_time} {units},"
        f" expected at least: {time_in} {units}"
    )
    return result, total_time


async def check_hold(signals, timeout, units="ns"):
    """Checks if each signal in `signals` holds their value for at least `timeout`."""

    async def check_all_hold(conditions):
        while True:
            for s, value in conditions:
                if s.value != value:
                    return
            await NextTimeStep()

    conditions = [(s, s.value) for s in signals]

    try:
        await with_timeout(check_all_hold(conditions), timeout, units)
    except SimTimeoutError:
        # `SimTimeoutError` was raised and therefore `conditions` were held
        # for at least `timeout`. Return gracefully.
        return

    # Otherwise, the `conditions` held for not enough time
    raise SimTimeoutError(
        f"Expected {signals}: to be held for at least {timeout} {units}",
        f"Changed: {[s for (s, v) in conditions if s.value != v]}",
    )


def report_config(speed: float, timings: I3cTimings, log_method: Callable[[str], None]) -> str:
    if isinstance(timings, I3cControllerTimings):
        mode = "Controller"
    elif isinstance(timings, I3cTargetTimings):
        mode = "Target"

    log_method(f"I3C {mode} configuration:")
    log_method(f"  Rate: {speed / 1000.0}kHz " f"({100.0 * speed / FULL_SPEED}%)")
    log_method("  Timings:")
    log_method(f"    SCL Clock High Period: {scaled_timing(timings.tdig_h, speed)}ns")
    log_method(f"    SCL Clock Low Period: {scaled_timing(timings.tdig_l, speed)}ns")
    log_method(f"    Clock After START (S) Condition: {scaled_timing(timings.tcas, speed)}ns")
    log_method(f"    Clock Before STOP (P) Condition: {scaled_timing(timings.tcbp, speed)}ns")
    log_method(
        f"    Clock Before Repeated START (Sr) Condition: {scaled_timing(timings.tcbsr, speed)}ns"
    )
    log_method(
        f"    Clock After Repeated START (Sr) Condition: {scaled_timing(timings.tcasr, speed)}ns"
    )
    log_method(f"    Bus Free condition: {scaled_timing(timings.tfree, speed)}ns")
    log_method(f"    Open-drain set-up time: {scaled_timing(timings.tsu_od, speed)}ns")
    log_method(f"    SDA Set-up time (Push-Pull): {scaled_timing(timings.tsupp, speed)}ns")
    log_method(f"    SDA Hold time (Push-Pull): {scaled_timing(timings.thd, speed)}ns")
    log_method(f"    Clock in to Data Out for Target: {scaled_timing(timings.tsco, speed)}ns")


async def with_timeout_event(event, trigger, timeout_in_ns, precision=(100, "ps")):
    result = None
    time = timeout_in_ns * 10
    while event.is_set() and time and result is None:
        time -= 1
        try:
            result = await with_timeout(trigger, *precision)
        except SimTimeoutError:
            pass

    return result


class I3cTargetResetAction(IntEnum):
    NO_RESET = 0x00
    RESET_PERIPHERAL_ONLY = 0x01
    RESET_WHOLE_TARGET = 0x02
    DEBUG_NETWORK_ADAPTER_RESET = 0x03
    VIRTUAL_TARGET_DETECT = 0x04


@dataclass
class I3cPRResp:
    nack: bool
    data: bytearray


@dataclass
class I3cPWResp:
    nack: bool
