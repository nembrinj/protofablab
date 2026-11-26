#!/usr/bin/env python3
"""OpenCV window with the annotated camera image from person_detection.

The training numbers (repetitions, heart rate, feedback, ...) are shown on
the Node-RED dashboard; this window only shows what the camera sees.

Input: /person/preview/compressed. Press q in the window to close it.
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# Camera frames: only the newest frame matters, dropping old ones is fine.
CAMERA_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
WINDOW_NAME = "Adaptive Training Assistant - camera"


class CameraViewNode(Node):
    def __init__(self):
        super().__init__("camera_view")
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.create_subscription(
            CompressedImage, "/person/preview/compressed", self.image_callback, CAMERA_QOS
        )

    def image_callback(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        cv2.imshow(WINDOW_NAME, frame)
        if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q")):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraViewNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C or q
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
