#!/usr/bin/env python

"""
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
"""

from cocotb.runner import get_results
from pathlib import Path
import sys

def main():
    results_file = Path('results.xml')
    (_, num_failed) = get_results(results_file)
    rc = 1 if num_failed else 0
    if rc:
        print("get_results: test(s) failed.")
    else:
        print("get_results: test(s) succeeded.")
    sys.exit(rc)

if __name__ == "__main__":
    main()
