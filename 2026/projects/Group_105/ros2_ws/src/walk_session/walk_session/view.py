"""
The session panel drawn in RViz: the walk in progress and the last walk.

The panel is drawn in the walking_path frame, which lidar_person_tracker
publishes: x along the path, y away from it. It sits beside the robot, on the
side away from the path, so it does not cover the walker.

This module has no ROS imports, so it can be tested without ROS.
"""

import math

FRAME = 'walking_path'  # published by lidar_person_tracker
POSITION_M = (0.0, 0.9)  # where the panel goes in FRAME
TEXT_HEIGHT_M = 0.16
TEXT_RGB = (1.0, 0.85, 0.4)  # yellow, apart from the tracker's white


def panel_text(session_id, progress, last_walk, stop_cue):
    """Return the panel's lines; progress and last_walk are None when there is none."""
    lines = [f'Session {session_id}']
    if progress is None:
        lines.append('Waiting for a walk to start')
    else:
        state = 'stopped' if progress.stopped else 'walking'
        lines.append(f'Walk {progress.number}: {state}, {count(progress.steps, "step")}, '
                     f'{count(progress.stops, "stop")}')
        if progress.stopped:
            lines.append(f'Said: "{stop_cue}"')
    if last_walk is not None:
        lines.append(f'Last walk ({last_walk.number}): {walk_numbers(last_walk)}')
    return '\n'.join(lines)


def walk_numbers(walk):
    """Return the numbers of a walk that has ended, on one line."""
    if not walk.measured:
        return f'not measured, {walk.duration_s:.1f} s'
    return ', '.join([
        f'{walk.duration_s:.1f} s',
        count(walk.steps, 'step'),
        known(walk.cadence_spm, '.0f', 'steps/min'),
        known(walk.speed_mps, '.2f', 'm/s'),
        count(walk.stops, 'stop'),
    ])


def count(number, thing):
    return f'{number} {thing}' + ('' if number == 1 else 's')


def known(value, spec, unit):
    return f'? {unit}' if math.isnan(value) else f'{value:{spec}} {unit}'
