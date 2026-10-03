#!/usr/bin/env python3
"""Loopback HTTP API and browser entry point; hardware-free preview by default."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation.http_api import main

if __name__ == '__main__':
    main()
