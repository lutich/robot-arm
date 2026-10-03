#!/usr/bin/env python3
"""service: see docs/deployment.md for usage."""
import sys
from pi import main

if __name__ == "__main__":
    main(["service", *sys.argv[1:]])
