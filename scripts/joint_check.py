#!/usr/bin/env python3
"""Explicit single-joint commissioning check; preview is hardware-free."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation.joint_check import main

if __name__ == '__main__':
    main()
