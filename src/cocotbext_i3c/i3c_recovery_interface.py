# SPDX-License-Identifier: Apache-2.0

import logging
import random

import crc
from cocotb.triggers import Timer

from .i3c_controller import I3cController


class I3cRecoveryException(RuntimeError):
    """
    A generic recovery-related exception class
    """

    pass


class I3cRecoveryInterface:
    """
    Recovery interface adapter to I3C controller
    """

    # PEC checksum calculator config
    # This should be equivalent to crc.CCIT8 but is left as explicit config
    # for possible changes.
    CRC_CONFIG = crc.Configuration(
        width=8,
        polynomial=0x7,
        init_value=0x00,
        final_xor_value=0x00,
        reverse_input=False,
        reverse_output=False,
    )

    # Command codes. As per OCP recovery spec
    class Command:
        PROT_CAP = 34
        DEVICE_ID = 35
        DEVICE_STATUS = 36
        DEVICE_RESET = 37
        RECOVERY_CTRL = 38
        RECOVERY_STATUS = 39
        HW_STATUS = 40
        INDIRECT_CTRL = 41
        INDIRECT_STATUS = 42
        INDIRECT_DATA = 43
        VENDOR = 44
        INDIRECT_FIFO_CTRL = 45
        INDIRECT_FIFO_STATUS = 46
        INDIRECT_FIFO_DATA = 47

    def __init__(self, controller: I3cController):
        self.log = logging.getLogger(f"cocotb.{controller.sda_o._path}")
        self.log.setLevel("DEBUG")
        self.controller = controller

        # Initialize PEC calculator
        self.pec_calc = crc.Calculator(self.CRC_CONFIG, optimized=True)

    @staticmethod
    def _randomize_pec(pec):
        """
        Returns a random value for PEC checksum not equal to the given one
        """
        while True:
            r = random.randint(0, 255)
            if r != pec:
                return r

    async def _i3c_recovery_read(self, address, send_stop=True, abort_after_bytes=None):
        """
        Issues a private read using low-level functions of the controller
        adapter. This is needed as the length of data to be received is
        contained in the first two bytes of the packet
        """

        await self.controller.send_start()
        ack = await self.controller.write_addr_header(address, read=True)
        if not ack:
            await self.controller.send_stop()
            self.controller.give_bus_control()
            return None, None

        # Begin reception
        try:
            # Read length
            len_bytes = []
            for i in range(2):
                abort_here = abort_after_bytes is not None and i == abort_after_bytes - 1
                byte, stop = await self.controller.recv_byte_t_bit(stop=abort_here)
                len_bytes.append(byte)
                self.log.debug(f"Recovery Rx: byte[{i}]: 0x{byte:02X} (stop={int(stop)})")

                # Length is mandatory. If the transfer gets terminated raise an
                # exception.
                if stop and not abort_here:
                    self.log.error(f"Target requested stop at byte {i}")
                    raise I3cRecoveryException
                elif stop:
                    if send_stop:
                        await self.controller.send_stop(pull_scl_low=False)
                        self.controller.give_bus_control()
                    return

            length = (len_bytes[1] << 8) | len_bytes[0]
            self.log.debug(f"Recovery Rx: Payload length is {length}B")

            # Read data. Raise an exception in case of an unexpected stop
            data = []
            for i in range(length):
                abort_here = abort_after_bytes is not None and i + 2 == abort_after_bytes - 1
                byte, stop = await self.controller.recv_byte_t_bit(stop=abort_here)
                data.append(byte)
                self.log.debug(f"Recovery Rx: byte[{i + 2}]: 0x{byte:02X} (stop={int(stop)})")

                if stop and not abort_here:
                    self.log.error(f"Target requested stop at byte {i + 2}")
                    raise I3cRecoveryException
                elif stop:
                    if send_stop:
                        await self.controller.send_stop(pull_scl_low=False)
                        self.controller.give_bus_control()
                    return

            # Read PEC. Expect stop at this byte
            pec_recv, stop = await self.controller.recv_byte_t_bit(stop=True)
            if not stop:
                self.log.error("Target wishes to transfer more data after PEC")
                raise I3cRecoveryException

        # In case of a protocol error wait some time and then re-throw
        except I3cRecoveryException:
            await Timer(1, "us")
            raise

        if send_stop:
            await self.controller.send_stop(pull_scl_low=abort_after_bytes is not None)
            self.controller.give_bus_control()

        # Compute reference PEC checksum
        pec_calc = int(self.pec_calc.checksum(bytes([(address << 1) | 1] + len_bytes + data)))

        # Return the data and received PEC validity
        return data, (pec_recv == pec_calc)

    async def command_write(
        self,
        address,
        command,
        data=None,
        force_pec_error=False,
        skip_pec=False,
        extra_pec_bytes=0,
        stop=True,
        start=True,
        rstart=False,
        claimed_length=None,
        abort_after_bytes=None,
        error_byte_index=None,
    ):
        """
        Issues a write command to the target
        """

        if not data:
            data = []

        length = len(data)
        if claimed_length is not None:
            length = claimed_length

        # Header
        xfer = [
            command,
            length & 0xFF,
            (length >> 8) & 0xFF,
        ]

        # Data
        xfer.extend(data)

        # Compute PEC
        pec = int(self.pec_calc.checksum(bytes([address << 1] + xfer)))

        # Inject incorrect PEC
        if force_pec_error:
            pec = self._randomize_pec(pec)

        if not skip_pec:
            xfer.append(pec)
            for _ in range(extra_pec_bytes):
                xfer.append(random.randint(0, 0xFF))

        if abort_after_bytes is not None:
            xfer = xfer[:abort_after_bytes]

        # Do the I3C write transfer using the controller functionality
        await self.controller.i3c_write(
            address,
            xfer,
            stop=stop,
            send_rsvd=start,
            rstart=rstart,
            take_bus_control=start,
            inject_tbit_err_on=error_byte_index,
        )

    async def command_read(
        self,
        address,
        command,
        force_pec_error=False,
        stop=True,
        start=True,
        error_byte_index=None,
        abort_after_bytes=None,
    ):
        """
        Issues a read command to the target
        """

        # Header
        xfer = [command]

        # Compute PEC
        pec = int(self.pec_calc.checksum(bytes([address << 1] + xfer)))

        # Inject incorrect PEC
        if force_pec_error:
            pec = self._randomize_pec(pec)

        xfer.append(pec)

        # Do the I3C write transfer, do not terminate with stop
        await self.controller.i3c_write(
            address, xfer, stop=False, send_rsvd=start, inject_tbit_err_on=error_byte_index
        )

        # Do the I3C read transfer. Return the results.
        return await self._i3c_recovery_read(
            address, send_stop=stop, abort_after_bytes=abort_after_bytes
        )
