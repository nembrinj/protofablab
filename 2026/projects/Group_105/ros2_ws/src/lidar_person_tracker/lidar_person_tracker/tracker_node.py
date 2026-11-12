"""ROS 2 node: LiDAR scans in, the state of the person on the walking path out."""

from dataclasses import fields

from gait_interfaces.msg import PersonState
from lidar_person_tracker.tracker import PersonTracker, TrackerConfig
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


class TrackerNode(Node):

    def __init__(self):
        super().__init__('lidar_person_tracker')
        # A wrong parameter raises here and stops the node at start-up.
        config = TrackerConfig(**{
            field.name: self.declare_parameter(field.name, field.default).value
            for field in fields(TrackerConfig)
        })
        self._scan_timeout_s = self.declare_parameter('scan_timeout_s', 2.0).value
        self._tracker = PersonTracker(config)
        self._ready = False
        self._last_scan = self.get_clock().now()

        self._publisher = self.create_publisher(PersonState, '/person/state', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '~/relearn_background', self._on_relearn)
        self.create_timer(1.0, self._check_scans)

    def _on_scan(self, scan):
        self._last_scan = self.get_clock().now()
        stamp_s = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        try:
            estimate = self._tracker.update(
                stamp_s, scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max)
        except ValueError as error:
            self.get_logger().error(f'Scan ignored: {error}', throttle_duration_sec=5.0)
            return
        if not estimate.ready:
            self.get_logger().info(
                'Learning the empty room, keep the walking path clear',
                throttle_duration_sec=2.0)
            return
        if not self._ready:
            self._ready = True
            self.get_logger().info('Empty room learned, now watching the walking path')

        state = PersonState()
        state.header = scan.header
        state.detected = estimate.detected
        state.people_count = min(estimate.people_count, 255)
        state.position_m = estimate.position_m
        state.speed_mps = estimate.speed_mps
        self._publisher.publish(state)

    def _on_relearn(self, request, response):
        self._tracker.relearn()
        self._ready = False
        response.success = True
        response.message = 'Learning the empty room again'
        self.get_logger().info(response.message)
        return response

    def _check_scans(self):
        silent_s = (self.get_clock().now() - self._last_scan).nanoseconds * 1e-9
        if silent_s > self._scan_timeout_s:
            self.get_logger().warn(
                f'No LiDAR scan for {silent_s:.0f} s, the person state is not updated',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
