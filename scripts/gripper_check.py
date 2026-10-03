#!/usr/bin/env python3
"""Single gripper check. Default is a hardware-free preview; see confirmation-test.md."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation.gripper_check import main

if __name__ == '__main__':
    main()
