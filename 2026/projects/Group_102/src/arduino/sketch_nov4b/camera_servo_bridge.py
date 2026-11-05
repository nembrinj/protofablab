import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class CameraServoBridge(Node):
    def __init__(self):
        super().__init__("camera_servo_bridge")

        # ESP32 configuration
        self.esp32_ip = "192.168.1.9"
        self.esp32_port = 4210

        # Maximum servo speed (degrees/second)
        self.max_velocity = 60.0

        # UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ROS 2 subscriber
        self.subscription = self.create_subscription(
            Float32, "/camera_servo/velocity", self.velocity_callback, 10
        )

        self.get_logger().info(
            f"Camera Servo Bridge started. "
            f"Sending UDP to {self.esp32_ip}:{self.esp32_port}"
        )

    def velocity_callback(self, msg: Float32):
        velocity = max(-self.max_velocity, min(self.max_velocity, float(msg.data)))

        # Send velocity as an ASCII number
        message = f"{velocity:.2f}"

        try:
            self.udp_socket.sendto(
                message.encode("utf-8"), (self.esp32_ip, self.esp32_port)
            )

            self.get_logger().info(f"Sent servo velocity: {message} deg/s")

        except OSError as error:
            self.get_logger().error(f"UDP communication error: {error}")

    def destroy_node(self):
        # Send stop command before shutting down
        try:
            self.udp_socket.sendto(b"0.00", (self.esp32_ip, self.esp32_port))
            self.udp_socket.close()
        except OSError:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraServoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
