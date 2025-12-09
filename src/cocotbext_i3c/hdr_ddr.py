"""
Copyright (c) 2025 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0

HDR-DDR (High Data Rate - Double Data Rate) support for I3C Controller
"""


def calculate_hdr_crc5(data: bytes) -> int:
    """
    Calculate CRC-5 for HDR-DDR mode.
    Polynomial: x^5 + x^2 + 1

    Args:
        data: Bytes to calculate CRC over

    Returns:
        5-bit CRC value
    """
    crc = 0x1F
    poly = 0x05

    for byte in data:
        for i in range(7, -1, -1):
            bit = (byte >> i) & 1
            msb = (crc >> 4) & 1
            crc = (crc << 1) & 0x1F
            if msb != bit:
                crc ^= poly
    return crc


def calculate_parity(byte: int) -> int:
    """Calculate odd parity bit for a byte."""
    parity = 1
    while byte:
        parity ^= byte & 1
        byte >>= 1
    return parity
