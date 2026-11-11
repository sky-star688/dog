"""OpenMV fine-positioning node for Dog 2.

Reads OpenMV USB telemetry (published by a host bridge as String) and
publishes fine-position estimates:

  @OVM:<cx>,<cy>,<width>,<height>,<pixels>!

Output JSON includes pixel centre, bbox size, stable count and fine position
in the arm-base frame (millimetres, requires calibration).
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OpenMVPositioner(Node):
    def __init__(self):
        super().__init__("openmv_positioner_dog2")

        self.declare_parameter("openmv_telemetry_topic", "/openmv/telemetry")
        self.declare_parameter("output_topic", "/dog2/openmv_position")
        self.declare_parameter("pixel_width", 320)
        self.declare_parameter("pixel_height", 240)
        self.declare_parameter("target_min_width", 20)
        self.declare_parameter("target_max_width", 200)
        self.declare_parameter("mm_per_pixel", 0.5)
        self.declare_parameter("arm_offset_x_mm", 100.0)
        self.declare_parameter("arm_offset_y_mm", 0.0)
        self.declare_parameter("arm_offset_z_mm", 50.0)
        self.declare_parameter("stable_frames", 5)

        self.pw = int(self.get_parameter("pixel_width").value)
        self.ph = int(self.get_parameter("pixel_height").value)
        self.min_w = int(self.get_parameter("target_min_width").value)
        self.max_w = int(self.get_parameter("target_max_width").value)
        self.mmpp = float(self.get_parameter("mm_per_pixel").value)
        self.ox = float(self.get_parameter("arm_offset_x_mm").value)
        self.oy = float(self.get_parameter("arm_offset_y_mm").value)
        self.oz = float(self.get_parameter("arm_offset_z_mm").value)
        self.need = int(self.get_parameter("stable_frames").value)

        self._latest = None
        self._stable = 0
        self._locked = False

        self.pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("openmv_telemetry_topic").value),
            self._on_telemetry,
            10,
        )
        self.create_timer(0.5, self._publish)
        self.get_logger().info("Dog 2 OpenMV positioner ready")

    def _on_telemetry(self, message: String) -> None:
        raw = message.data.strip()
        if not raw.startswith("@OVM:") or not raw.endswith("!"):
            return
        try:
            parts = raw[5:-1].split(",")
            if len(parts) < 5:
                return
            cx = float(parts[0])
            cy = float(parts[1])
            w = float(parts[2])
            h = float(parts[3])
            pixels = int(parts[4])
        except (ValueError, IndexError):
            return
        if w < self.min_w or w > self.max_w:
            self._locked = False
            self._stable = 0
            self._latest = None
            return
        if self._latest is not None:
            dx = abs(cx - float(self._latest["cx"]))
            dy = abs(cy - float(self._latest["cy"]))
            if dx < 10.0 and dy < 10.0:
                self._stable += 1
            else:
                self._stable = max(0, self._stable - 1)
        self._latest = {
            "cx": cx,
            "cy": cy,
            "w": w,
            "h": h,
            "pixels": pixels,
            "ts": time.time(),
        }
        if self._stable >= self.need:
            self._locked = True

    def _publish(self) -> None:
        if not self._locked or self._latest is None:
            return
        t = self._latest
        dx = float(t["cx"]) - self.pw / 2.0
        dy = float(t["cy"]) - self.ph / 2.0
        fx = self.ox + dx * self.mmpp
        fy = self.oy - dy * self.mmpp
        fz = self.oz
        msg = String()
        msg.data = json.dumps(
            {
                "source": "openmv",
                "target_locked": True,
                "pixel_cx": t["cx"],
                "pixel_cy": t["cy"],
                "pixel_width": t["w"],
                "pixel_height": t["h"],
                "pixels": int(t["pixels"]),
                "fine_x_mm": fx,
                "fine_y_mm": fy,
                "fine_z_mm": fz,
                "approx_dist_mm": (self.max_w * self.mmpp)
                / max(float(t["w"]), 1.0),
                "stable_count": self._stable,
                "timestamp": t["ts"],
            }
        )
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenMVPositioner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
