# Changelog

This document describes changes to the cocotbext-i3c repository.

## 1.1.0

Added:
  * Model of the I3C Target:
    * Memory model connected to the target.
    * Recognizing STOP and START conditions.
    * Recognizing and responding (ACK) to the RESERVED BYTE header.
    * Handling private reads; terminating message after issued read byte to the controller.
    * Handling private writes; verifying odd-parity bit after data blocks.
    * Ignoring non-directed and CCC transfers.
  * Common I3C file - [common.py](src/cocotbext_i3c/common.py) and moved common definitions:
    * includes `I3cState`, `I3cTimings` and time-measuring helper functions
  * Tests:
    * Moved Testbench setup to a common file.
    * Added tests for Target & Controller connected with `i3c_harness`:
      * private read-only,
      * private write-only,
      * read-write sequence with `STOP` issued in between operations,
      * read-write sequence without the `STOP` condition,
      * read-write sequence without the `STOP` condition, interlaced with operations to other devices.

Modified:
  * Project setup:
    * added configuration for `black`, `isort` and `flake8`,
    * added `pytest` configuration to add `tests/` to initial paths and allow imports from common files.

Fixed:
  * Clock after data timing for START.
    Assigning state of the I3C at the beginning of the `send_start` function caused `bus_active` to always be `True` and caused `tCAS` timing to always be `tCASr`.

## 1.0.0

Added:
  * model of the I3C Controller
