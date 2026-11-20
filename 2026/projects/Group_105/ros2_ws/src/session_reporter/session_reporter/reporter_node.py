"""
ROS 2 node: walk summaries in, JSON messages to the dashboard's MQTT broker out.

Walks wait in a queue while the broker cannot be reached, and are sent once it
can, so a walk is not lost when the dashboard starts late or Wi-Fi drops.
"""

from collections import deque

from gait_interfaces.msg import WalkSummary
import paho.mqtt.client as mqtt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from session_reporter.reporter import walk_payload, walk_topic

CONNECT_S = 3.0  # time to connect before a missing broker is reported


def make_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  # paho-mqtt 2
    except AttributeError:
        return mqtt.Client()  # paho-mqtt 1, as in Ubuntu 24.04


class ReporterNode(Node):

    def __init__(self):
        super().__init__('session_reporter')
        self._host = self.declare_parameter('broker_host', 'localhost').value
        self._port = self.declare_parameter('broker_port', 1883).value
        # A wrong patient_id raises here and stops the node at start-up.
        self._topic = walk_topic(self.declare_parameter('patient_id', 'p01').value)
        self._waiting = deque()  # (walk number, JSON text), oldest first
        self._connected = False
        self._started = self.get_clock().now()

        self._client = make_client()
        self._client.reconnect_delay_set(min_delay=1, max_delay=10)
        self._client.connect_async(self._host, self._port)
        self._client.loop_start()  # keeps trying to connect in the background

        self.create_subscription(WalkSummary, '/session/walk_summary', self._on_walk, 10)
        self.create_timer(1.0, self._send_waiting)
        self.get_logger().info(f'Sending walks to {self._topic} on {self._host}:{self._port}')

    def _on_walk(self, summary):
        self._waiting.append((summary.walk, walk_payload(summary)))
        self._send_waiting()

    def _send_waiting(self):
        connected = self._client.is_connected()
        if connected != self._connected:
            self._connected = connected
            if connected:
                self.get_logger().info(
                    f'Connected to the MQTT broker at {self._host}:{self._port}')
            else:
                self.get_logger().warn(
                    f'Lost the MQTT broker at {self._host}:{self._port}, trying again')
        while self._waiting and connected:
            walk, payload = self._waiting[0]
            result = self._client.publish(self._topic, payload, qos=1)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warn(f'Walk {walk} not sent yet: {mqtt.error_string(result.rc)}')
                return
            self._waiting.popleft()
            self.get_logger().info(f'Walk {walk} sent to the dashboard')
        trying_s = (self.get_clock().now() - self._started).nanoseconds * 1e-9
        if not connected and trying_s > CONNECT_S:
            self.get_logger().warn(
                f'No MQTT broker at {self._host}:{self._port}; is the dashboard running? '
                f'{len(self._waiting)} walk(s) waiting', throttle_duration_sec=10.0)

    def destroy_node(self):
        if self._waiting:
            self.get_logger().error(
                f'{len(self._waiting)} walk(s) never reached the dashboard: '
                + ', '.join(str(walk) for walk, _ in self._waiting))
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReporterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # stopped with Ctrl+C, or by the launch file
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
