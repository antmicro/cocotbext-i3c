#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

from pathlib import Path

from cocotb.runner import check_results_file

if __name__ == "__main__":
    results_file = Path("results.xml")
    check_results_file(results_file)
