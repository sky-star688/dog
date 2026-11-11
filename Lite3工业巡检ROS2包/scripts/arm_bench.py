#!/usr/bin/env python3
"""Bench test for MyArm-2 serial (no ROS).

Accepts only semantic actions and prints the ASCII frame by default.
``--execute`` sends one frame after an interactive SEND confirmation.
No completion ACK exists; never treat a write as physical completion.
"""

import argparse
import sys

try:
    import serial
except ImportError:
    serial = None


def build_frame(action, pick_group, place_group):
    if action == "home":
        return b"$DJR!"
    if action == "stop":
        return b"$DST!"
    if action == "status":
        return None
    if action == "pick":
        return b"$DGS:%d!" % pick_group
    if action == "place":
        return b"$DGS:%d!" % place_group
    raise ValueError("unsupported action: %s" % action)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["status", "home", "stop", "pick", "place"])
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--pick-group", type=int, default=1)
    parser.add_argument("--place-group", type=int, default=2)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    frame = build_frame(args.action, args.pick_group, args.place_group)
    print("action=%s frame=%r" % (args.action, frame))

    if not args.execute or frame is None:
        print("dry-run: no serial bytes written")
        return 0

    if serial is None:
        print("ERROR pyserial is not installed")
        return 1
    if not args.yes:
        answer = input("type SEND to write one frame: ")
        if answer.strip() != "SEND":
            print("aborted")
            return 1

    with serial.Serial(args.port, args.baudrate, timeout=0.2) as port:
        port.write(frame)
        port.flush()
    print("frame written (no completion ACK available)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
