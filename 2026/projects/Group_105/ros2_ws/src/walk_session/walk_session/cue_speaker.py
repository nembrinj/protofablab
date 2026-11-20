"""
ROS 2 node: speaks the session's cues on the laptop.

A stand-in until audio_cues speaks them on the robot's speaker. It hands each
text to a speech command, by default spd-say, which comes with Ubuntu Desktop.
"""

import shlex
import shutil
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

COMMAND_TIMEOUT_S = 5.0


class CueSpeaker(Node):

    def __init__(self):
        super().__init__('cue_speaker')
        self._command = shlex.split(self.declare_parameter('speech_command', 'spd-say').value)
        if not self._command or shutil.which(self._command[0]) is None:
            raise RuntimeError(f'speech command {self._command} not found; '
                               'install speech-dispatcher or set speech_command')
        self.create_subscription(String, '/session/cue', self._on_cue, 10)
        self.get_logger().info(f'Speaking the cues with {self._command[0]}')

    def _on_cue(self, cue):
        try:
            subprocess.run(self._command + [cue.data], check=True, capture_output=True,
                           timeout=COMMAND_TIMEOUT_S)
        except (OSError, subprocess.SubprocessError) as error:
            self.get_logger().error(f'Could not speak "{cue.data}": {error}')


def main(args=None):
    rclpy.init(args=args)
    node = CueSpeaker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # stopped with Ctrl+C, or by the launch file
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
