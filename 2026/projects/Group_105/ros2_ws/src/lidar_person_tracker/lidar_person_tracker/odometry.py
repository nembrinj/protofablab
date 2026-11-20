"""
Where the robot was at a given time, from its odometry.

The odometry adds up the wheel turns since the robot started: a position and
a heading in a fixed frame, called odom. It drifts slowly, but over a few
seconds it is exact enough to take the robot's own movement out of the scans.

This module has no ROS imports, so it can be tested without ROS.
"""

from collections import deque
from dataclasses import dataclass
import math

KEEP_S = 2.0  # how much odometry is kept
MAX_GAP_S = 0.25  # no pose is given further than this from an odometry message


@dataclass(frozen=True)
class Pose:
    """Where the robot is in the odom frame."""

    x_m: float
    y_m: float
    yaw_rad: float  # heading, counter-clockwise from the odom x axis

    def moved_from(self, other):
        """Return how far (m) and how much (rad) the robot moved and turned since other."""
        turn = math.atan2(math.sin(self.yaw_rad - other.yaw_rad),
                          math.cos(self.yaw_rad - other.yaw_rad))
        return math.hypot(self.x_m - other.x_m, self.y_m - other.y_m), abs(turn)


class PoseHistory:
    """The robot's recent poses, to look up where it was when a scan was taken."""

    def __init__(self):
        self._poses = deque()  # (time, Pose), oldest first

    def add(self, stamp_s, pose):
        poses = self._poses
        if poses and stamp_s < poses[-1][0]:
            poses.clear()  # the time went back, for example a recording started again
        if poses and stamp_s == poses[-1][0]:
            poses.pop()
        poses.append((stamp_s, pose))
        while stamp_s - poses[0][0] > KEEP_S:
            poses.popleft()

    def latest(self):
        """Return the newest Pose, or None before the first odometry message."""
        return self._poses[-1][1] if self._poses else None

    def at(self, stamp_s):
        """Return the Pose at that time, between the two nearest messages; None if unknown."""
        poses = self._poses
        if not poses or stamp_s < poses[0][0] - MAX_GAP_S or stamp_s > poses[-1][0] + MAX_GAP_S:
            return None
        if stamp_s <= poses[0][0]:
            return poses[0][1]
        if stamp_s >= poses[-1][0]:
            return poses[-1][1]
        for (start_s, start), (end_s, end) in zip(poses, list(poses)[1:]):
            if start_s <= stamp_s <= end_s:
                share = (stamp_s - start_s) / (end_s - start_s)
                turn = math.atan2(math.sin(end.yaw_rad - start.yaw_rad),
                                  math.cos(end.yaw_rad - start.yaw_rad))
                return Pose(start.x_m + share * (end.x_m - start.x_m),
                            start.y_m + share * (end.y_m - start.y_m),
                            start.yaw_rad + share * turn)
        return None
