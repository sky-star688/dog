#!/usr/bin/env python3
"""Offline pixel -> arm-plane homography calibration from JSON points.

Input JSON: {"pixel_points": [[u,v],...], "arm_points": [[x_mm,y_mm],...],
             "workspace_mm": {"x": [min,max], "y": [min,max], "z": [min,max]}}
Output JSON: {"homography": [[..]], "workspace_mm": {...}, "rms_mm": ...}
At least four non-collinear points are required.
"""

import argparse
import json
import sys

from lite3_inspection.openmv import homography_error, solve_homography


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("points_json")
    parser.add_argument("--output", default="calibration.json")
    parser.add_argument("--max-rms-mm", type=float, default=2.0)
    args = parser.parse_args()

    with open(args.points_json, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    pixels = data.get("pixel_points", [])
    arms = data.get("arm_points", [])
    matrix = solve_homography(pixels, arms)
    rms, worst = homography_error(matrix, pixels, arms)

    payload = {"homography": matrix, "rms_mm": rms, "max_error_mm": worst}
    if "workspace_mm" in data:
        payload["workspace_mm"] = data["workspace_mm"]

    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)

    print("rms=%.3f mm worst=%.3f mm -> %s" % (rms, worst, args.output))
    if rms > args.max_rms_mm:
        print("FAIL residual exceeds --max-rms-mm=%.2f; re-collect points" % args.max_rms_mm)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
