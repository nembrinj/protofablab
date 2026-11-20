"""ROS 2 node: LiDAR scans and odometry in, the state of the person on the walking path out."""

from dataclasses import fields
import math

from gait_interfaces.msg import PersonState
from lidar_person_tracker.tracker import MODES, PersonTracker, TrackerConfig
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(orientation):
    """Return the heading of a quaternion, counter-clockwise from the x axis."""
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


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
        self._tracker.set_mode(self.declare_parameter(
            'mode', 'static', ParameterDescriptor(
                description=f'{" or ".join(MODES)}; can be changed while the node runs')).value)
        self.add_on_set_parameters_callback(self._on_parameters)
        self._ready = False
        self._following = False
        self._last_scan = self.get_clock().now()
        self._last_odometry = None

        self._publisher = self.create_publisher(PersonState, '/person/state', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odometry, qos_profile_sensor_data)
        self.create_service(Trigger, '~/relearn_background', self._on_relearn)
        self.create_timer(1.0, self._check_inputs)
        self.get_logger().info(f'Mode: {self._tracker.mode}')

    def _on_parameters(self, parameters):
        fixed = {field.name for field in fields(TrackerConfig)} | {'scan_timeout_s'}
        for parameter in parameters:
            if parameter.name in fixed:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} is read at start-up; restart the node to change it')
            if parameter.name != 'mode':
                continue
            try:
                self._tracker.set_mode(parameter.value)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
            self.get_logger().info(f'Mode: {parameter.value}')
        return SetParametersResult(successful=True)

    def _on_odometry(self, odometry):
        self._last_odometry = self.get_clock().now()
        pose = odometry.pose.pose
        try:
            self._tracker.add_odometry(stamp_seconds(odometry.header), pose.position.x,
                                       pose.position.y, yaw_of(pose.orientation))
        except ValueError as error:
            self.get_logger().error(f'Odometry ignored: {error}', throttle_duration_sec=5.0)

    def _on_scan(self, scan):
        self._last_scan = self.get_clock().now()
        stamp_s = stamp_seconds(scan.header)
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
        if estimate.following != self._following:
            self._following = estimate.following
            self.get_logger().info('Following the walker' if estimate.following
                                   else 'Walker lost, no longer following')
        if estimate.moved:
            self.get_logger().warn(
                'The robot has moved since it learned the empty room, so nobody is reported. '
                'Park it, clear the path, and call relearn_background',
                throttle_duration_sec=5.0)

        state = PersonState()
        state.header = scan.header
        state.detected = estimate.detected
        state.people_count = min(estimate.people_count, 255)
        state.position_m = estimate.position_m
        state.speed_mps = estimate.speed_mps
        state.steps = estimate.steps
        self._publisher.publish(state)

    def _on_relearn(self, request, response):
        self._tracker.relearn()
        self._ready = False
        response.success = True
        response.message = 'Learning the empty room again'
        self.get_logger().info(response.message)
        return response

    def _check_inputs(self):
        now = self.get_clock().now()
        silent_s = (now - self._last_scan).nanoseconds * 1e-9
        if silent_s > self._scan_timeout_s:
            self.get_logger().warn(
                f'No LiDAR scan for {silent_s:.0f} s, the person state is not updated',
                throttle_duration_sec=5.0)
        if self._tracker.mode == 'driving' and (
                self._last_odometry is None
                or (now - self._last_odometry).nanoseconds * 1e-9 > self._scan_timeout_s):
            self.get_logger().warn('Driving mode needs /odom, and none is arriving',
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # stopped with Ctrl+C, or by the launch file
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
