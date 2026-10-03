#!/usr/bin/env python3
"""SSH askpass helper (ROBOT_ASKPASS=1) and pi CLI entry point; stdlib-only askpass path."""
import os
import sys

if __name__ == '__main__':
    if os.environ.get('ROBOT_ASKPASS') == '1':
        if len(sys.argv) != 2 or 'password:' not in sys.argv[1].lower():
            raise SystemExit(1)
        print(os.environ['ROBOT_SSH_PASSWORD'])
    else:
        from pi import main
        main()
