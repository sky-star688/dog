"""Dog 1 vision detector – reuses lite3_inspection.VisionDetector.

Runs under Dog 1's namespace.  The shared detector publishes 2-D detections;
dog1_depth_projector adds 3-D map points.
"""

from lite3_inspection.vision_detector import VisionDetector


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
