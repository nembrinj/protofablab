"""
ROS 2 node: the walker's state in, walk summaries and spoken cues out.

Also shows a panel of the session in RViz (see view.py).
"""

from dataclasses import fields
import time

from gait_interfaces.msg import PersonState, WalkSummary
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA, String
from walk_session import view
from walk_session.session import Seen, SessionConfig, WalkSession


class SessionNode(Node):

    def __init__(self):
        super().__init__('walk_session')
        # A wrong parameter raises here and stops the node at start-up.
        config = SessionConfig(**{
            field.name: self.declare_parameter(field.name, field.default).value
            for field in fields(SessionConfig)
        })
        self._stop_cue = self.declare_parameter(
            'stop_cue', 'Take your time, start again when ready').value
        self._state_timeout_s = self.declare_parameter('state_timeout_s', 2.0).value
        self._session = WalkSession(config)
        # The laptop's clock: the robot's clock may be wrong.
        self._session_id = time.strftime('%Y%m%d-%H%M%S')
        self._last_state = self.get_clock().now()

        self._summaries = self.create_publisher(WalkSummary, '/session/walk_summary', 10)
        self._cues = self.create_publisher(String, '/session/cue', 10)
        self._panel = self.create_publisher(OverlayText, '~/panel', 10)
        self.create_subscription(PersonState, '/person/state', self._on_state, 10)
        self.create_timer(1.0, self._check_states)
        self.get_logger().info(f'Session {self._session_id} started, waiting for the walker')

    def _on_state(self, state):
        self._last_state = self.get_clock().now()
        stamp_s = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
        seen = Seen(stamp_s, state.people_count, state.position_m, state.speed_mps, state.steps)
        try:
            update = self._session.update(seen)
        except ValueError as error:
            self._session.forget_walk()
            self.get_logger().error(f'Walk in progress dropped: {error}')
            return
        if update.ended:
            self._publish_walk(update.ended, state.header)
        if update.started:
            self.get_logger().info(f'Walk {update.started} started')
        if update.stopped:
            self.get_logger().info(f'Walk {self._session.walks}: the walker stopped')
            self._cues.publish(String(data=self._stop_cue))
        if self._panel.get_subscription_count() > 0:
            self._panel.publish(self._draw())

    def _draw(self):
        """Return the panel with the walk in progress, in the top left."""
        lines = view.panel_lines(self._session.progress, self._session.walks)
        return OverlayText(
            action=OverlayText.ADD,
            width=view.PANEL_WIDTH_PX, height=view.panel_height_px(lines),
            horizontal_alignment=OverlayText.LEFT, vertical_alignment=OverlayText.TOP,
            horizontal_distance=view.PANEL_MARGIN_PX, vertical_distance=view.PANEL_MARGIN_PX,
            bg_color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.7),
            fg_color=ColorRGBA(r=1.0, g=0.9, b=0.6, a=1.0),
            text_size=float(view.PANEL_TEXT_PT), text=view.panel_text(lines))

    def _publish_walk(self, walk, header):
        summary = WalkSummary()
        summary.header = header
        summary.session_id = self._session_id
        summary.walk = walk.number
        summary.measured = walk.measured
        summary.duration_s = walk.duration_s
        summary.steps = walk.steps
        summary.cadence_spm = walk.cadence_spm
        summary.speed_mps = walk.speed_mps
        summary.stops = walk.stops
        self._summaries.publish(summary)
        self.get_logger().info(
            f'Walk {walk.number} ended: {"measured" if walk.measured else "not measured"}, '
            f'{walk.duration_s:.1f} s, {walk.steps} steps, {walk.cadence_spm:.0f} steps/min, '
            f'{walk.speed_mps:.2f} m/s, {walk.stops} stops')

    def destroy_node(self):
        if self._session.walk_in_progress:
            self.get_logger().warn(
                f'Walk {self._session.walk_in_progress} had not ended; it is not sent')
        super().destroy_node()

    def _check_states(self):
        silent_s = (self.get_clock().now() - self._last_state).nanoseconds * 1e-9
        if silent_s > self._state_timeout_s:
            self.get_logger().warn(
                f'No walker state for {silent_s:.0f} s; is lidar_person_tracker running?',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = SessionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # stopped with Ctrl+C, or by the launch file
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
