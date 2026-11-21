"""
What the tracker draws in RViz: the watched strip, the people's points, and a status line.

Everything is drawn in the walking_path frame: the LiDAR's frame, turned so
that x runs along the path, the same way as position_m, and y points away
from the path. Seen from above with x to the right, the path then lies
across the screen below the robot, whichever way the robot faces it.

This module has no ROS imports, so it can be tested without ROS.
"""

import math

import numpy as np

FRAME = 'walking_path'
STATUS_GAP_M = 0.4  # from the far edge of the strip to the status line
TEXT_HEIGHT_M = 0.16

STRIP_RGB = (0.3, 0.6, 1.0)  # blue
WALKER_RGB = (0.2, 0.9, 0.3)  # green: one person, measured
CROWD_RGB = (1.0, 0.55, 0.1)  # orange: more than one person, not measured
TEXT_RGB = (1.0, 1.0, 1.0)


def frame_yaw(facing_angle_rad):
    """Return how far the walking_path frame is turned from the LiDAR's frame."""
    return facing_angle_rad + math.pi / 2


def to_frame(points_m):
    """Turn (along, across) rows, from the robot, into (x, y) rows in walking_path."""
    points = np.asarray(points_m, dtype=float).reshape(-1, 2)
    return np.column_stack((points[:, 0], -points[:, 1]))


def strip_outline(config):
    """Return the corners of the watched strip in walking_path, as a closed line."""
    near = -(config.path_distance_m - config.corridor_half_width_m)
    far = -(config.path_distance_m + config.corridor_half_width_m)
    half = config.zone_half_length_m
    return [(-half, near), (half, near), (half, far), (-half, far), (-half, near)]


def status_position(config):
    """Return where the status line goes in walking_path: below the middle of the strip."""
    return (0.0, -(config.path_distance_m + config.corridor_half_width_m + STATUS_GAP_M))


def people_rgb(estimate):
    """Return the colour of the people's points."""
    return WALKER_RGB if estimate.people_count <= 1 else CROWD_RGB


def status_text(estimate, mode):
    """Return two lines on what the tracker sees in this scan."""
    if not estimate.ready:
        return 'Learning the empty room\nkeep the walking path clear'
    if estimate.moved:
        return 'The robot has moved, so nobody is reported\npark it, clear the path, relearn'
    if estimate.people_count == 1:
        speed = 'speed not known yet' if math.isnan(estimate.speed_mps) \
            else f'{abs(estimate.speed_mps):.2f} m/s'
        seen = f'Walker at {estimate.position_m:+.1f} m, {speed}'
    elif estimate.people_count > 1:
        seen = f'{estimate.people_count} people on the path, not measured'
    elif estimate.following:
        seen = 'Walker out of sight for a moment'
    else:
        seen = 'Nobody on the path'
    details = f'{estimate.steps} step{"" if estimate.steps == 1 else "s"}, {mode} mode'
    if estimate.following:
        details += ', following'
    return f'{seen}\n{details}'
