"""
What the tracker shows in RViz: the watched strip, the people's points, and a status panel.

The strip and the points are drawn in the walking_path frame: the LiDAR's
frame, turned so that x runs along the path, the same way as position_m, and
y points away from the path. Seen from above with x to the right, the path
then lies across the screen below the robot, whichever way the robot faces it.

The status is a text panel fixed in the top right corner of the RViz view
(rviz_2d_overlay_plugins): RViz's own text markers draw spaces far too wide.

This module has no ROS imports, so it can be tested without ROS.
"""

import html
import math

import numpy as np

FRAME = 'walking_path'

STRIP_RGB = (0.3, 0.6, 1.0)  # blue
WALKER_RGB = (0.2, 0.9, 0.3)  # green: one person, measured
CROWD_RGB = (1.0, 0.55, 0.1)  # orange: more than one person, not measured

# The status panel, in pixels of the RViz view.
PANEL_WIDTH_PX = 200
PANEL_MARGIN_PX = 10  # from the edges of the view
PANEL_LINE_PX = 20
PANEL_TEXT_PT = 11


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


def people_rgb(estimate):
    """Return the colour of the people's points."""
    return WALKER_RGB if estimate.people_count <= 1 else CROWD_RGB


def status_lines(estimate, mode):
    """Return the lines of the status panel: what the tracker sees in this scan, in few words."""
    if not estimate.ready:
        return ['Learning the room']
    if estimate.moved:
        return ['Robot moved: relearn']
    if estimate.people_count == 1:
        seen = f'{estimate.position_m:+.1f} m'
        if not math.isnan(estimate.speed_mps):
            seen += f', {abs(estimate.speed_mps):.2f} m/s'
    elif estimate.people_count > 1:
        seen = f'{estimate.people_count} people'
    elif estimate.following:
        seen = 'Out of sight'
    else:
        seen = 'Nobody'
    lines = [seen, f'{estimate.steps} step{"" if estimate.steps == 1 else "s"} total']
    if mode == 'driving':
        lines.append('Driving, following' if estimate.following else 'Driving')
    return lines


def panel_text(lines):
    """Join the lines for the panel, which reads its text as HTML."""
    return '\n'.join(html.escape(line, quote=False) for line in lines)


def panel_height_px(lines):
    return len(lines) * PANEL_LINE_PX + PANEL_LINE_PX // 2
