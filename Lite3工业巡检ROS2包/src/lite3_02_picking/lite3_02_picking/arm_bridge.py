"""Dog 2 arm bridge – reuses lite3_inspection.ArmSerialBridge.

Run with Dog 2's port/topics through config/picking.yaml or remaps.
"""

from lite3_inspection.arm_serial_bridge import ArmSerialBridge


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = ArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
