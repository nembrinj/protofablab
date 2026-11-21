"""
ROS 2 node: LiDAR scans and odometry in, the state of the person on the walking path out.

Also shows what it sees in RViz: markers in the walking_path frame, and a status panel
(see view.py).
"""

from dataclasses import fields
import math

from gait_interfaces.msg import PersonState
from geometry_msgs.msg import Point, TransformStamped
from lidar_person_tracker import view
from lidar_person_tracker.tracker import MODES, PersonTracker, TrackerConfig
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rviz_2d_overlay_msgs.msg import OverlayText
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


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
        self._config = config
        self._tracker = PersonTracker(config)
        self._tracker.set_mode(self.declare_parameter(
            'mode', 'static', ParameterDescriptor(
                description=f'{" or ".join(MODES)}; can be changed while the node runs')).value)
        self.add_on_set_parameters_callback(self._on_parameters)
        self._ready = False
        self._following = False
        self._last_scan = self.get_clock().now()
        self._last_odometry = None
        self._scan_frame = None  # the frame walking_path was published in

        self._publisher = self.create_publisher(PersonState, '/person/state', 10)
        self._markers = self.create_publisher(MarkerArray, '~/markers', 10)
        self._panel = self.create_publisher(OverlayText, '~/panel', 10)
        self._frames = StaticTransformBroadcaster(self)
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
        self._publish_frame(scan.header.frame_id)
        if self._markers.get_subscription_count() > 0:
            self._markers.publish(self._draw(scan.header.stamp, estimate))
        if self._panel.get_subscription_count() > 0:
            self._panel.publish(self._status_panel(estimate))
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

    def _publish_frame(self, scan_frame):
        """Place the walking_path frame on the LiDAR's frame, once per LiDAR frame."""
        if scan_frame == self._scan_frame:
            return
        self._scan_frame = scan_frame
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = scan_frame
        transform.child_frame_id = view.FRAME
        yaw = view.frame_yaw(self._config.facing_angle_rad)
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        self._frames.sendTransform(transform)

    def _draw(self, stamp, estimate):
        """Return the markers that show what the tracker sees in this scan."""
        strip = self._marker(stamp, 'strip', Marker.LINE_STRIP, view.STRIP_RGB)
        strip.scale.x = 0.02
        strip.points = [Point(x=x, y=y) for x, y in view.strip_outline(self._config)]
        people = self._marker(stamp, 'people', Marker.POINTS, view.people_rgb(estimate))
        people.scale.x = people.scale.y = 0.07
        people.points = [Point(x=float(x), y=float(y))
                         for x, y in view.to_frame(estimate.points_m)]
        if not people.points:
            people.action = Marker.DELETE
        return MarkerArray(markers=[strip, people])

    def _status_panel(self, estimate):
        """Return the status panel, in the top right corner of the RViz view."""
        lines = view.status_lines(estimate, self._tracker.mode)
        return OverlayText(
            action=OverlayText.ADD,
            width=view.PANEL_WIDTH_PX, height=view.panel_height_px(lines),
            horizontal_alignment=OverlayText.RIGHT, vertical_alignment=OverlayText.TOP,
            horizontal_distance=view.PANEL_MARGIN_PX, vertical_distance=view.PANEL_MARGIN_PX,
            bg_color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.7),
            fg_color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
            text_size=float(view.PANEL_TEXT_PT), text=view.panel_text(lines))

    @staticmethod
    def _marker(stamp, name, kind, rgb):
        marker = Marker(ns=name, type=kind, action=Marker.ADD)
        marker.header.stamp = stamp
        marker.header.frame_id = view.FRAME
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
        return marker

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
