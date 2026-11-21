"""Checks of what the tracker draws in RViz."""

import math

from lidar_person_tracker import view
from lidar_person_tracker.tracker import PersonEstimate, TrackerConfig
import numpy as np
import pytest


def in_walking_path(x_m, y_m, facing_angle_rad):
    """Turn a point of the LiDAR's frame into the walking_path frame, as RViz does with the TF."""
    yaw = view.frame_yaw(facing_angle_rad)
    return (x_m * math.cos(yaw) + y_m * math.sin(yaw),
            -x_m * math.sin(yaw) + y_m * math.cos(yaw))


@pytest.mark.parametrize('facing_angle_rad', [0.0, -math.pi / 2, math.pi])
def test_the_frame_and_the_points_agree(facing_angle_rad):
    # 2 m towards the path, then 0.5 m along it, towards the robot's left.
    across = np.array([math.cos(facing_angle_rad), math.sin(facing_angle_rad)])
    along = np.array([-math.sin(facing_angle_rad), math.cos(facing_angle_rad)])
    x_m, y_m = 2.0 * across + 0.5 * along
    assert in_walking_path(x_m, y_m, facing_angle_rad) == pytest.approx((0.5, -2.0))
    assert view.to_frame([[0.5, 2.0]]).tolist() == [[0.5, -2.0]]


def test_no_points_give_an_empty_array():
    assert view.to_frame(np.empty((0, 2))).shape == (0, 2)


def test_the_strip_lies_below_the_robot():
    outline = view.strip_outline(TrackerConfig())
    assert outline[0] == outline[-1]
    assert {x for x, _ in outline} == {-2.5, 2.5}
    assert {y for _, y in outline} == {-0.5, -3.5}
    assert view.status_position(TrackerConfig())[1] < -3.5


def estimate(people_count=1, position_m=0.4, speed_mps=0.72, steps=26, ready=True,
             following=False, moved=False):
    return PersonEstimate(ready, people_count, position_m, speed_mps, steps, 0.3, following,
                          moved)


@pytest.mark.parametrize('seen, mode, text', [
    (estimate(), 'static', 'Walker at +0.4 m, 0.72 m/s\n26 steps, static mode'),
    (estimate(speed_mps=-0.72), 'static', 'Walker at +0.4 m, 0.72 m/s\n26 steps, static mode'),
    (estimate(speed_mps=math.nan), 'static',
     'Walker at +0.4 m, speed not known yet\n26 steps, static mode'),
    (estimate(people_count=2), 'static',
     '2 people on the path, not measured\n26 steps, static mode'),
    (estimate(people_count=0), 'static', 'Nobody on the path\n26 steps, static mode'),
    (estimate(people_count=0, steps=1), 'static', 'Nobody on the path\n1 step, static mode'),
    (estimate(following=True), 'driving',
     'Walker at +0.4 m, 0.72 m/s\n26 steps, driving mode, following'),
    (estimate(people_count=0, following=True), 'driving',
     'Walker out of sight for a moment\n26 steps, driving mode, following'),
])
def test_status_text(seen, mode, text):
    assert view.status_text(seen, mode) == text


def test_status_text_while_learning_and_after_a_move():
    assert view.status_text(estimate(ready=False), 'static').startswith('Learning the empty room')
    assert view.status_text(estimate(people_count=0, moved=True), 'static') \
        .startswith('The robot has moved')


def test_more_than_one_person_is_drawn_in_another_colour():
    assert view.people_rgb(estimate()) == view.WALKER_RGB
    assert view.people_rgb(estimate(people_count=2)) == view.CROWD_RGB
