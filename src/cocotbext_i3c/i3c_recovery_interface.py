# SPDX-License-Identifier: Apache-2.0

import random
import logging

import crc

from .i3c_controller import I3cController
from .common import I3C_RSVD_BYTE


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

    async def _i3c_recovery_read(self, address):
        """
        Issues a private read using low-level functions of the controller
        adapter. This is needed as the length of data to be received is
        contained in the first two bytes of the packet
        """

        # Begin I3C read
        await self.controller.take_bus_control()

        await self.controller.send_start()
        await self.controller.write_addr_header(I3C_RSVD_BYTE)
        await self.controller.send_start()
        await self.controller.write_addr_header(address, read=True)

        prtocol_error = False

        # Read length
        len_bytes = []
        for i in range(2):
            byte, stop = await self.controller.recv_byte_t_bit(stop=False)
            len_bytes.append(byte)
            self.log.debug(f"Recovery Rx: byte[{i}]: 0x{byte:02X} (stop={int(stop)})")

            # Length is mandatory. If the transfer gets terminated raise an
            # exception.
            if stop:
                self.log.error(f"Target requested stop at byte {i}")
                protocol_error = True
                #self.controller.give_bus_control()
                #raise I3cRecoveryException

        length = (len_bytes[1] << 8) | len_bytes[0]

        # Read data. Raise an exception in case of an unexpected stop
        data = []
        for i in range(length):
            byte, stop = await self.controller.recv_byte_t_bit(stop=False)
            data.append(byte)

            if stop:
                self.log.error(f"Target requested stop at byte {i+2}")
                protocol_error = True
                #self.controller.give_bus_control()
                #raise I3cRecoveryException

        # Read PEC. Expect stop at this byte
        pec_recv, stop = await self.controller.recv_byte_t_bit(stop=True)
        if not stop:
            self.log.error(f"Target wishes to transfer more data after PEC")
            protocol_error = True
            #self.controller.give_bus_control()
            #raise I3cRecoveryException

        self.controller.give_bus_control()

        if protocol_error:
            raise I3cRecoveryException

        # Compute reference PEC checksum
        pec_calc = int(self.pec_calc.checksum(bytes([(address << 1) | 1] + len_bytes + data)))

        # Return the data and received PEC validity
        return data, (pec_recv == pec_calc)

    async def command_write(self, address, command, data=None, force_pec_error=False):
        """
        Issues a write command to the target
        """

        if not data:
            data = []

        # Header
        xfer = [
            command,
            len(data) & 0xFF,
            (len(data) >> 8) & 0xFF,
        ]

        # Data
        xfer.extend(data)

        # Compute PEC
        pec = int(self.pec_calc.checksum(bytes([address << 1] + xfer)))

        # Inject incorrect PEC
        if force_pec_error:
            pec = self._randomize_pec(pec)

        xfer.append(pec)

        # Do the I3C write transfer using the controller functionality
        await self.controller.i3c_write(address, xfer)

    async def command_read(self, address, command, force_pec_error=False):
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
        await self.controller.i3c_write(address, xfer, stop=False)

        # Do the I3C read transfer. Return the results.
        return await self._i3c_recovery_read(address)
