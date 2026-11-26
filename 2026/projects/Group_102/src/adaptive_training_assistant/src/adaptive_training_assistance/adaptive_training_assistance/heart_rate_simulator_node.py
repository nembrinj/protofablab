#!/usr/bin/env python3
"""Dummy heart-rate sensor: publishes a simulated heart rate on /heart_rate.

Stands in for the real sensor until it is connected. It sends one value per
second: START_BPM before the exercise, and from the moment the exercise
starts (status IN_PROGRESS) it rises by RISE_PER_SECOND up to MAX_BPM. The
real sensor node only has to publish the same message (std_msgs/Float32,
bpm) on the same topic.

Input:  /training/status (only to know when the exercise starts)
Output: /heart_rate (Float32, bpm)
"""

import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

# Latched value: a node that starts later still receives the last message.
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# ---- Scenario: keep ONE block active, comment out the other two. ----------

# 1) Low heart rate that does not rise
START_BPM = 55.0
RISE_PER_SECOND = 0.0
MAX_BPM = 55.0

# 2) Heart rate that rises steadily (normal)
# START_BPM = 70.0
# RISE_PER_SECOND = 0.5
# MAX_BPM = 130.0

# 3) Starts normal, then rises much faster than usual (and above 150)
# START_BPM = 70.0
# RISE_PER_SECOND = 3.0
# MAX_BPM = 180.0
# ---------------------------------------------------------------------------

PUBLISH_PERIOD = 1.0  # A real chest strap sends about one value per second.
NOISE_BPM = 1.0  # Small random variation, like a real sensor.


class HeartRateSimulatorNode(Node):
    def __init__(self):
        super().__init__("heart_rate_simulator")
        self.start_time = None  # When the exercise started (None = not running).
        self.heart_rate_pub = self.create_publisher(Float32, "/heart_rate", 10)
        self.create_subscription(String, "/training/status", self.status_callback, LATCHED_QOS)
        self.create_timer(PUBLISH_PERIOD, self.publish_heart_rate)
        self.get_logger().info(
            f"Simulated heart rate: {START_BPM:.0f} bpm, then from the exercise "
            f"start +{RISE_PER_SECOND:.1f} bpm/s, up to {MAX_BPM:.0f} bpm"
        )

    def status_callback(self, msg):
        if msg.data == "IN_PROGRESS" and self.start_time is None:
            self.start_time = time.monotonic()  # Exercise started: start rising.
        elif msg.data != "IN_PROGRESS":
            self.start_time = None  # Exercise over: back to START_BPM.

    def publish_heart_rate(self):
        seconds = 0.0 if self.start_time is None else time.monotonic() - self.start_time
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
