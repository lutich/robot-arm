#!/usr/bin/env python3
"""deploy: see docs/deployment.md for usage."""
import sys
from pi import main

if __name__ == "__main__":
    main(["deploy", *sys.argv[1:]])
