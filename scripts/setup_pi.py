#!/usr/bin/env python3
"""setup-pi: see docs/deployment.md for usage."""
import sys
from pi import main

if __name__ == "__main__":
    main(["setup-pi", *sys.argv[1:]])
