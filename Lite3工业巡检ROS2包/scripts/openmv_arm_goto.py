#!/usr/bin/env python3
"""One-shot OpenMV -> MyArm KMS goto (bench tool).

Default dry-run.  With --execute it sends exactly one $KMS frame after an
interactive SEND confirmation.  It never sends $DGS and must not run while
arm_serial_bridge holds the same serial port.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    serial = None

from lite3_inspection.openmv import (
    extract_frames,
    load_calibration_file,
    target_to_kms,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--openmv-port", default="/dev/ttyACM0")
    parser.add_argument("--arm-port", default="/dev/ttyUSB0")
    parser.add_argument("--homography", required=True)
    parser.add_argument("--z-mm", type=float, required=True)
    parser.add_argument("--time-ms", type=int, default=500)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    if serial is None:
        print("ERROR pyserial is not installed")
        return 1

    matrix, workspace = load_calibration_file(args.homography)
    if args.execute and workspace is None:
        print("ERROR calibration has no workspace_mm; refusing to execute")
        return 1

    remainder = b""
    target = None
    deadline = time.time() + 10.0
    with serial.Serial(args.openmv_port, 115200, timeout=0.2) as camera:
        while time.time() < deadline and target is None:
            chunk = camera.read(camera.in_waiting or 1)
            if not chunk:
                continue
            targets, _, remainder = extract_frames(remainder + chunk)
            if targets:
                target = targets[-1]

    if target is None:
        print("no stable @OVM frame received")
        return 1

    x, y, frame = target_to_kms(target, matrix, args.z_mm, args.time_ms, workspace)
    print("target -> arm (%.1f, %.1f) frame=%r" % (x, y, frame))

    if not args.execute:
        print("dry-run: no arm bytes written")
        return 0
    if not args.yes:
        if input("type SEND to write one $KMS frame: ").strip() != "SEND":
            print("aborted")
            return 1

    with serial.Serial(args.arm_port, 115200, timeout=0.2) as arm:
        arm.write(frame)
        arm.flush()
    print("$KMS written (no completion ACK available)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
