#!/usr/bin/env python3
"""Connection, setup, deployment, the app service and passive checks."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.provisioning.presentation.cli import main

if __name__ == '__main__':
    main()
