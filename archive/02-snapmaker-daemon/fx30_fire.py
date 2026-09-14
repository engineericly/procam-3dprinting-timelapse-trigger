#!/usr/bin/env python3
"""
fx30_fire.py - tiny client for fx30_daemon.py

This is what the Klipper gcode_shell_command actually runs. It just opens
the daemon's Unix socket, sends one word, and reports the result - all the
real work (the PTP/IP session) already happened once, at daemon start-up,
and stays open in the background. This script's whole job is to be fast:
connect, ask, get an answer, exit.

Exit code 0 = shutter fired. Exit code 1 = it didn't (camera not ready,
daemon not running, etc) - the error is printed to stderr so it shows up
in Klipper's console/log if RUN_SHELL_COMMAND is set to report failures.
"""

import socket
import sys
import os

SOCK_PATH = os.environ.get("FX30_SOCK_PATH", "/run/fx30-trigger/trigger.sock")
TIMEOUT_S = 3.0


def main():
    cmd = sys.argv[1].upper() if len(sys.argv) > 1 else "FIRE"
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(TIMEOUT_S)
            s.connect(SOCK_PATH)
            s.sendall((cmd + "\n").encode())
            reply = s.recv(256).decode(errors="replace").strip()
    except (OSError, socket.timeout) as e:
        print("fx30_fire: could not reach daemon at %s (%s) - is "
              "fx30-trigger.service running?" % (SOCK_PATH, e), file=sys.stderr)
        sys.exit(1)

    if reply == "OK" or reply in ("READY", "LOST", "CONNECTING"):
        print(reply)
        sys.exit(0)
    print("fx30_fire: %s" % reply, file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    main()
