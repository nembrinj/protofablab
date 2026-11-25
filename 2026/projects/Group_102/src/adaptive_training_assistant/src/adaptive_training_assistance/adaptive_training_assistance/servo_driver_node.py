#!/usr/bin/env python3
"""Send the camera-servo angle to the ESP32 over UDP.

Input: /servo/angle (degrees, from servo_tilt). Each angle is sent as text,
e.g. "72.50". The ESP32 just moves the servo to it, so if messages stop the
servo simply stays where it is.
"""

import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

SERVO_IP = "172.20.10.4"
SERVO_PORT = 4210


class ServoDriverNode(Node):
    def __init__(self):
        super().__init__("servo_driver")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Float32, "/servo/angle", self.angle_callback, 10)
        self.get_logger().info(f"Servo driver ready. ESP32={SERVO_IP}:{SERVO_PORT}")

    def angle_callback(self, msg):
        try:
            self.socket.sendto(f"{msg.data:.2f}".encode(), (SERVO_IP, SERVO_PORT))
        except OSError as exc:
            self.get_logger().error(f"Could not send UDP command: {exc}")

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
