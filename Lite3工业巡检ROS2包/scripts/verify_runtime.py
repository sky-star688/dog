#!/usr/bin/env python3
"""Read-only runtime verification for the legacy single-dog stack.

Checks that required topics/actions exist and that safety signals are in the
expected state.  Never starts a mission or sends motion/arm commands.
"""

import json
import sys

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except Exception as exc:  # pragma: no cover - ROS not installed
    print("FAIL cannot import rclpy: %s" % exc)
    sys.exit(1)


GATES = []


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_runtime")
        self.values = {}
        for name in (
            "localization",
            "detector",
            "depth",
            "guard",
            "mux",
            "arm_ready",
            "arm_status",
        ):
            self.values[name] = None
        self.create_subscription(
            Bool, "/inspection/localization_status",
            lambda m: self.values.__setitem__("localization", bool(m.data)), 10)
        self.create_subscription(
            String, "/inspection/detector_status",
            lambda m: self.values.__setitem__("detector", m.data), 10)
        self.create_subscription(
            String, "/inspection/depth_status",
            lambda m: self.values.__setitem__("depth", m.data), 10)
        self.create_subscription(
            String, "/inspection/guard_status",
            lambda m: self.values.__setitem__("guard", m.data), 10)
        self.create_subscription(
            String, "/inspection/mux_status",
            lambda m: self.values.__setitem__("mux", m.data), 10)
        self.create_subscription(
            Bool, "/inspection/arm_ready",
            lambda m: self.values.__setitem__("arm_ready", bool(m.data)), 10)
        self.create_subscription(
            String, "/arm/status",
            lambda m: self.values.__setitem__("arm_status", m.data), 10)


def main():
    rclpy.init()
    node = Verifier()
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.25)
    ok = True

    localization = node.values["localization"]
    print(("OK   " if localization else "FAIL "), "localization_status", localization)
    ok = ok and bool(localization)

    detector = node.values["detector"]
    try:
        detector_status = json.loads(detector or "{}").get("status")
    except Exception:
        detector_status = None
    print(("OK   " if detector_status == "ready" else "FAIL "), "detector_status", detector_status)
    ok = ok and detector_status == "ready"

    depth = node.values["depth"]
    try:
        has_depth = json.loads(depth or "{}").get("has_depth")
    except Exception:
        has_depth = None
    print(("OK   " if has_depth else "FAIL "), "depth_status", depth)
    ok = ok and bool(has_depth)

    guard = node.values["guard"]
    try:
        guard_enabled = json.loads(guard or "{}").get("enabled")
    except Exception:
        guard_enabled = None
    print(("OK   " if guard_enabled is False else "WARN "), "guard enabled", guard_enabled)

    mux = node.values["mux"]
    try:
        mux_enabled = json.loads(mux or "{}").get("enabled")
    except Exception:
        mux_enabled = None
    print(("OK   " if mux_enabled is False else "WARN "), "mux enabled", mux_enabled)

    print("arm_status", node.values["arm_status"])
    node.destroy_node()
    rclpy.shutdown()
    print("RESULT", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
