#!/usr/bin/env python3
"""Append immutable calibration evidence; never command hardware."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.calibration.presentation.cli import main

if __name__ == '__main__':
    main()
