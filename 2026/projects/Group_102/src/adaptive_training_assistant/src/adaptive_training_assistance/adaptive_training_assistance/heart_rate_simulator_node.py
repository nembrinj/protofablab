#!/usr/bin/env python3
"""Dummy heart-rate sensor: publishes a simulated heart rate on /heart_rate.

Stands in for the real sensor until it is connected. It listens to nothing:
it only sends one value per second, starting at START_BPM when the node
starts and rising by RISE_PER_SECOND until MAX_BPM. The real sensor node only
has to publish the same message (std_msgs/Float32, bpm) on the same topic.

Output: /heart_rate (Float32, bpm)
"""

import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

# ---- Scenario: keep ONE block active, comment out the other two. ----------

# 1) Low heart rate that does not rise
# START_BPM = 65.0
# RISE_PER_SECOND = 0.0
# MAX_BPM = 65.0

# 2) Heart rate that rises steadily (normal)
START_BPM = 70.0
RISE_PER_SECOND = 0.5
MAX_BPM = 130.0

# 3) Starts normal, then rises far too high (above 150)
# START_BPM = 70.0
# RISE_PER_SECOND = 2.0
# MAX_BPM = 180.0
# ---------------------------------------------------------------------------

PUBLISH_PERIOD = 1.0  # A real chest strap sends about one value per second.
NOISE_BPM = 1.0  # Small random variation, like a real sensor.


class HeartRateSimulatorNode(Node):
    def __init__(self):
        super().__init__("heart_rate_simulator")
        self.start_time = time.monotonic()
        self.heart_rate_pub = self.create_publisher(Float32, "/heart_rate", 10)
        self.create_timer(PUBLISH_PERIOD, self.publish_heart_rate)
        self.get_logger().info(
            f"Simulated heart rate: {START_BPM:.0f} bpm, "
            f"+{RISE_PER_SECOND:.1f} bpm/s, up to {MAX_BPM:.0f} bpm"
        )

    def publish_heart_rate(self):
        seconds = time.monotonic() - self.start_time
        heart_rate = min(MAX_BPM, START_BPM + RISE_PER_SECOND * seconds)
        msg = Float32()
        msg.data = float(round(heart_rate + random.gauss(0.0, NOISE_BPM), 1))
        self.heart_rate_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeartRateSimulatorNode()
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
