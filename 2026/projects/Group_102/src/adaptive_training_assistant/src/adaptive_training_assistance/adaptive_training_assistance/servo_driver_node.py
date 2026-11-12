#!/usr/bin/env python3
"""Send camera-servo velocity commands to the ESP32 over UDP."""

import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

COMMAND_TIMEOUT_SECONDS = 0.5
SERVO_IP = "172.20.10.4"
SERVO_PORT = 4210
SERVO_ENABLED = True


class ServoDriverNode(Node):
    def __init__(self):
        super().__init__("servo_driver")

        self.servo_ip = SERVO_IP
        self.servo_port = SERVO_PORT
        self.servo_enabled = SERVO_ENABLED

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_command_time = time.monotonic()
        self.watchdog_stopped = False

        self.subscription = self.create_subscription(
            Float32, "/servo/velocity", self.velocity_callback, 10
        )
        self.watchdog_timer = self.create_timer(0.1, self.check_watchdog)

        self.get_logger().info(
            f"Servo driver ready. ESP32={self.servo_ip}:{self.servo_port}, "
            f"enabled={self.servo_enabled}"
        )

    def send_udp(self, velocity):
        if not self.servo_enabled:
            return
        payload = f"{velocity:.2f}".encode("utf-8")
        try:
            self.socket.sendto(payload, (self.servo_ip, self.servo_port))
        except OSError as exc:
            self.get_logger().error(f"Could not send UDP command: {exc}")

    def velocity_callback(self, msg):
        velocity = max(-8.0, min(8.0, float(msg.data)))
        self.last_command_time = time.monotonic()
        self.watchdog_stopped = False
        self.send_udp(velocity)

    def check_watchdog(self):
        if (
            not self.watchdog_stopped
            and time.monotonic() - self.last_command_time > COMMAND_TIMEOUT_SECONDS
        ):
            self.send_udp(0.0)
            self.watchdog_stopped = True
            self.get_logger().warning(
                "No servo command received; sending stop command."
            )

    def destroy_node(self):
        self.send_udp(0.0)
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
