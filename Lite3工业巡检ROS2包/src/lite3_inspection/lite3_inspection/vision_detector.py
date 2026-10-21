import json
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisionDetector(Node):
    """Run an Ultralytics PT model or YOLOv8 ONNX model on a ROS image topic."""

    def __init__(self):
        super().__init__("vision_detector")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/inspection/detections")
        self.declare_parameter("annotated_topic", "/inspection/annotated_image")
        self.declare_parameter("model_path", "models/best.onnx")
        self.declare_parameter("labels_path", "models/labels.txt")
        self.declare_parameter("confidence", 0.55)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("input_size", 640)
        self.declare_parameter("process_every_n_frames", 2)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("device", "0")

        self.bridge = CvBridge()
        self.frame_number = 0
        self.backend = None
        self.model = None
        self.labels = self.load_labels(str(self.get_parameter("labels_path").value))
        self.load_model()

        self.detection_pub = self.create_publisher(
            String, str(self.get_parameter("detections_topic").value), 10
        )
        self.annotated_pub = self.create_publisher(
            Image, str(self.get_parameter("annotated_topic").value), 2
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )

    def load_labels(self, path):
        if not os.path.isfile(path):
            return []
        with open(path, "r", encoding="utf-8") as stream:
            return [line.strip() for line in stream if line.strip()]

    def load_model(self):
        path = os.path.expanduser(str(self.get_parameter("model_path").value))
        if not os.path.isfile(path):
            self.get_logger().error("Model does not exist: %s" % path)
            return
        suffix = os.path.splitext(path)[1].lower()
        if suffix == ".onnx":
            self.model = cv2.dnn.readNetFromONNX(path)
            self.backend = "opencv_onnx"
        else:
            try:
                from ultralytics import YOLO

                self.model = YOLO(path)
                self.backend = "ultralytics"
            except Exception as exc:
                self.get_logger().error("Cannot load Ultralytics model: %s" % exc)
                return
        self.get_logger().info("Vision model loaded with %s: %s" % (self.backend, path))

    def on_image(self, message):
        self.frame_number += 1
        every = max(1, int(self.get_parameter("process_every_n_frames").value))
        if self.frame_number % every != 0 or self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if self.backend == "ultralytics":
                detections = self.infer_ultralytics(frame)
            else:
                detections = self.infer_onnx(frame)
            self.publish_detections(message, frame, detections)
        except Exception as exc:
            self.get_logger().error("Vision inference failed: %s" % exc)

    def infer_ultralytics(self, frame):
        confidence = float(self.get_parameter("confidence").value)
        device = str(self.get_parameter("device").value)
        result = self.model.predict(frame, conf=confidence, device=device, verbose=False)[0]
        names = result.names
        detections = []
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            detections.append(
                self.make_detection(
                    class_id,
                    str(names[class_id]),
                    float(box.conf[0].item()),
                    x1,
                    y1,
                    x2,
                    y2,
                )
            )
        return detections

    def infer_onnx(self, frame):
        size = int(self.get_parameter("input_size").value)
        height, width = frame.shape[:2]
        scale = min(float(size) / width, float(size) / height)
        resized_w, resized_h = int(width * scale), int(height * scale)
        resized = cv2.resize(frame, (resized_w, resized_h))
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        pad_x, pad_y = (size - resized_w) // 2, (size - resized_h) // 2
        canvas[pad_y : pad_y + resized_h, pad_x : pad_x + resized_w] = resized
        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (size, size), swapRB=True)
        self.model.setInput(blob)
        output = np.squeeze(self.model.forward())
        if output.ndim != 2:
            raise RuntimeError("Unexpected ONNX output shape: %s" % (output.shape,))
        if output.shape[0] < output.shape[1]:
            output = output.transpose()

        boxes, scores, class_ids = [], [], []
        confidence_limit = float(self.get_parameter("confidence").value)
        for row in output:
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])
            if score < confidence_limit:
                continue
            center_x, center_y, box_w, box_h = row[:4]
            x = int((center_x - box_w / 2 - pad_x) / scale)
            y = int((center_y - box_h / 2 - pad_y) / scale)
            w = int(box_w / scale)
            h = int(box_h / scale)
            boxes.append([x, y, w, h])
            scores.append(score)
            class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(
            boxes,
            scores,
            confidence_limit,
            float(self.get_parameter("iou_threshold").value),
        )
        detections = []
        for index in np.array(indices).reshape(-1) if len(indices) else []:
            x, y, w, h = boxes[int(index)]
            x1, y1 = max(0, x), max(0, y)
            x2, y2 = min(width - 1, x + w), min(height - 1, y + h)
            class_id = class_ids[int(index)]
            label = self.labels[class_id] if class_id < len(self.labels) else str(class_id)
            detections.append(
                self.make_detection(class_id, label, scores[int(index)], x1, y1, x2, y2)
            )
        return detections

    @staticmethod
    def make_detection(class_id, label, score, x1, y1, x2, y2):
        return {
            "class_id": class_id,
            "class_name": label,
            "confidence": round(score, 4),
            "bbox": [x1, y1, x2, y2],
            "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
        }

    def publish_detections(self, source_message, frame, detections):
        payload = {
            "stamp": {
                "sec": source_message.header.stamp.sec,
                "nanosec": source_message.header.stamp.nanosec,
            },
            "frame_id": source_message.header.frame_id,
            "detections": detections,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.detection_pub.publish(message)

        if bool(self.get_parameter("publish_annotated").value):
            for detection in detections:
                x1, y1, x2, y2 = detection["bbox"]
                label = "%s %.2f" % (
                    detection["class_name"],
                    detection["confidence"],
                )
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(
                    frame,
                    label,
                    (x1, max(18, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 200, 255),
                    2,
                )
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated.header = source_message.header
            self.annotated_pub.publish(annotated)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
