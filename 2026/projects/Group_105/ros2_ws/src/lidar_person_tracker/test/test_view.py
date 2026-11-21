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


def estimate(people_count=1, position_m=0.4, speed_mps=0.72, steps=26, ready=True,
             following=False, moved=False):
    return PersonEstimate(ready, people_count, position_m, speed_mps, steps, 0.3, following,
                          moved)


@pytest.mark.parametrize('seen, mode, lines', [
    (estimate(), 'static', ['+0.4 m, 0.72 m/s', '26 steps total']),
    (estimate(speed_mps=-0.72), 'static', ['+0.4 m, 0.72 m/s', '26 steps total']),
    (estimate(speed_mps=math.nan), 'static', ['+0.4 m', '26 steps total']),
    (estimate(people_count=2), 'static', ['2 people', '26 steps total']),
    (estimate(people_count=0), 'static', ['Nobody', '26 steps total']),
    (estimate(people_count=0, steps=1), 'static', ['Nobody', '1 step total']),
    (estimate(people_count=0), 'driving', ['Nobody', '26 steps total', 'Driving']),
    (estimate(following=True), 'driving',
     ['+0.4 m, 0.72 m/s', '26 steps total', 'Driving, following']),
    (estimate(people_count=0, following=True), 'driving',
     ['Out of sight', '26 steps total', 'Driving, following']),
    (estimate(ready=False), 'static', ['Learning the room']),
    (estimate(people_count=0, moved=True), 'static', ['Robot moved: relearn']),
])
def test_status_lines(seen, mode, lines):
    assert view.status_lines(seen, mode) == lines


def test_the_panel_has_a_line_per_item_and_is_read_as_html():
    lines = ['a < b & c', 'second']
    assert view.panel_text(lines) == 'a &lt; b &amp; c\nsecond'
    assert view.panel_height_px(lines) > view.panel_height_px(lines[:1])


def test_more_than_one_person_is_drawn_in_another_colour():
    assert view.people_rgb(estimate()) == view.WALKER_RGB
    assert view.people_rgb(estimate(people_count=2)) == view.CROWD_RGB
