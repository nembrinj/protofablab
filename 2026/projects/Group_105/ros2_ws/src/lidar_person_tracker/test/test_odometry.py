"""Checks of looking up where the robot was at a given time."""

import math

from lidar_person_tracker.odometry import MAX_GAP_S, Pose, PoseHistory
import pytest


def history(*poses):
    result = PoseHistory()
    for stamp_s, pose in poses:
        result.add(stamp_s, pose)
    return result


def test_pose_between_two_messages():
    pose = history((1.0, Pose(0.0, 0.0, 0.0)), (1.1, Pose(0.1, -0.2, 0.2))).at(1.05)
    assert (pose.x_m, pose.y_m, pose.yaw_rad) == pytest.approx((0.05, -0.1, 0.1))


def test_heading_turns_the_short_way_round():
    pose = history((0.0, Pose(0.0, 0.0, 3.1)), (0.1, Pose(0.0, 0.0, -3.1))).at(0.05)
    assert abs(math.sin(pose.yaw_rad)) < 0.01
    assert math.cos(pose.yaw_rad) < 0


def test_no_pose_far_from_the_messages():
    poses = history((1.0, Pose(0.0, 0.0, 0.0)), (1.1, Pose(0.1, 0.0, 0.0)))
    assert poses.at(1.1 + MAX_GAP_S / 2) == Pose(0.1, 0.0, 0.0)
    assert poses.at(1.1 + 2 * MAX_GAP_S) is None
    assert poses.at(1.0 - 2 * MAX_GAP_S) is None
    assert PoseHistory().at(1.0) is None


def test_time_going_back_starts_again():
    poses = history((5.0, Pose(1.0, 0.0, 0.0)), (1.0, Pose(2.0, 0.0, 0.0)))
    assert poses.at(1.0) == Pose(2.0, 0.0, 0.0)
    assert poses.at(5.0) is None


def test_moved_from():
    distance, turn = Pose(3.0, 4.0, -3.0).moved_from(Pose(0.0, 0.0, 3.0))
    assert distance == pytest.approx(5.0)
    assert turn == pytest.approx(2 * math.pi - 6.0)
