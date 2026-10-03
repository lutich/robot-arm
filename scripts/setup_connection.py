#!/usr/bin/env python3
"""setup-connection: see docs/deployment.md for usage."""
import sys
from pi import main

if __name__ == "__main__":
    main(["setup-connection", *sys.argv[1:]])
