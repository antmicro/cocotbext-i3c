"""
Copyright (c) 2025 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0

HDR-BT (High Data Rate - Bulk Transfer) support for I3C Controller
"""


def calculate_hdr_crc16(data: bytes) -> int:
    """
    Calculate CRC-16 for HDR-BT mode.
    Polynomial: x^16 + x^15 + x^2 + 1

    Args:
        data: Bytes to calculate CRC over

    Returns:
        16-bit CRC value
    """
    crc = 0xFFFF
    poly = 0x8005

    for byte in data:
        for i in range(8):
            bit = (byte >> i) & 1
            msb = (crc >> 15) & 1
            crc = (crc << 1) & 0xFFFF
            if msb != bit:
                crc ^= poly
    # Reverse bits
    reverse = bin(crc)[-1:1:-1]
    reverse = reverse + (16 - len(reverse)) * "0"
    crc = int(reverse, 2)
    return crc


def calculate_hdr_crc32(data: bytes) -> int:
    """
    Calculate CRC-32 for HDR-BT mode.
    Polynomial: x^32 + X^26 + x^23 + x^22 x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1

    Args:
        data: Bytes to calculate CRC over

    Returns:
        32-bit CRC value
    """
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7

    for byte in data:
        for i in range(8):
            bit = (byte >> i) & 1
            msb = (crc >> 31) & 1
            crc = (crc << 1) & 0xFFFFFFFF
            if msb != bit:
                crc ^= poly
    # Reverse bits
    reverse = bin(crc)[-1:1:-1]
    reverse = reverse + (16 - len(reverse)) * "0"
    crc = int(reverse, 2)
    return crc
