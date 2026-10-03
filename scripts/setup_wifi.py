#!/usr/bin/env python3
"""setup-wifi: see docs/deployment.md for usage."""
import sys
from pi import main

if __name__ == "__main__":
    main(["setup-wifi", *sys.argv[1:]])
