#!/usr/bin/env python3
"""Read-only OpenMV @OVM telemetry monitor (no ROS, no arm access)."""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None

from lite3_inspection.openmv import apply_homography, extract_frames, load_calibration_file


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--homography", default="")
    args = parser.parse_args()

    if serial is None:
        print("ERROR pyserial is not installed")
        return 1

    matrix = None
    if args.homography:
        matrix, workspace = load_calibration_file(args.homography)
        print("loaded calibration; workspace=%s" % workspace)

    remainder = b""
    deadline = time.time() + args.seconds
    with serial.Serial(args.port, args.baudrate, timeout=0.2) as port:
        while time.time() < deadline:
            chunk = port.read(port.in_waiting or 1)
            if not chunk:
                continue
            targets, errors, remainder = extract_frames(remainder + chunk)
            for error in errors:
                print("bad frame:", error)
            for target in targets:
                line = "target cx=%d cy=%d w=%d h=%d px=%d" % (
                    target.center_x, target.center_y,
                    target.width, target.height, target.pixels,
                )
                if matrix is not None:
                    x, y = apply_homography(matrix, target.center_x, target.center_y)
                    line += " -> arm (%.1f, %.1f) mm" % (x, y)
                print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
