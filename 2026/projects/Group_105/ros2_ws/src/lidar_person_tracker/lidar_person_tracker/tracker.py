"""
Find a walking person in the LiDAR scans of a parked robot.

The robot does not move, so the empty room always gives the same scan. The
tracker first remembers that scan. After that, a point that is clearly nearer
than the empty room, and lies on the walking path, belongs to a person.

This module has no ROS imports, so it can be tested without ROS.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

BINS = 360  # one direction per degree, all around the robot


@dataclass(frozen=True)
class TrackerConfig:
    """Settings of the tracker; lengths in metres, times in seconds."""

    path_distance_m: float = 2.5
    corridor_half_width_m: float = 0.75
    zone_half_length_m: float = 2.5
    facing_angle_rad: float = 0.0
    background_scans: int = 15
    background_margin_m: float = 0.2
    person_gap_m: float = 0.8
    min_points: int = 2
    speed_window_s: float = 1.0
    min_speed_span_s: float = 0.4
    max_jump_m: float = 1.0

    def __post_init__(self):
        positive = [
            'path_distance_m', 'corridor_half_width_m', 'zone_half_length_m',
            'background_margin_m', 'person_gap_m', 'speed_window_s',
            'min_speed_span_s', 'max_jump_m',
        ]
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a positive number, got {value}')
        if not math.isfinite(self.facing_angle_rad):
            raise ValueError(f'facing_angle_rad must be a number, got {self.facing_angle_rad}')
        for name in ['background_scans', 'min_points']:
            value = getattr(self, name)
            if int(value) != value or value < 1:
                raise ValueError(f'{name} must be a whole number of at least 1, got {value}')
        if self.min_speed_span_s > self.speed_window_s:
            raise ValueError('min_speed_span_s must not be longer than speed_window_s')


@dataclass(frozen=True)
class PersonEstimate:
    """What one scan says about people on the walking path."""

    ready: bool  # False while the empty room is still being learned
    people_count: int
    position_m: float  # along the path; NaN unless exactly one person
    speed_mps: float  # along the path; NaN until enough scans of one person

    @property
    def detected(self):
        return self.people_count > 0


def bin_scan(ranges, angle_min, angle_increment, range_min, range_max):
    """
    Put a scan on a fixed grid of one range per degree.

    LiDAR models differ in how many beams they send, and some change the
    number from scan to scan. On a fixed grid, scans can be compared.
    A direction with no usable measurement gets infinity (nothing seen).
    """
    distances = np.asarray(ranges, dtype=float)
    if distances.ndim != 1 or distances.size == 0:
        raise ValueError('the scan has no ranges')
    if not math.isfinite(angle_min) or not math.isfinite(angle_increment) \
            or angle_increment == 0:
        raise ValueError('the scan angles are not usable')
    angles = angle_min + angle_increment * np.arange(distances.size)
    usable = np.isfinite(distances) & (distances >= range_min) & (distances <= range_max)
    index = np.round(np.degrees(angles[usable])).astype(int) % BINS
    binned = np.full(BINS, np.inf)
    np.minimum.at(binned, index, distances[usable])
    return binned


class PersonTracker:
    """Learns the empty room, then reports people on the walking path."""

    def __init__(self, config=None):
        self._config = config or TrackerConfig()
        self._angles = np.radians(np.arange(BINS)) - self._config.facing_angle_rad
        self._history = deque()  # (time, position) of the one person, newest last
        self.relearn()

    def relearn(self):
        """Forget the empty room and learn it again from the next scans."""
        self._background = None
        self._learning = []
        self._history.clear()

    def update(self, stamp_s, ranges, angle_min, angle_increment, range_min, range_max):
        """Take one scan and return a PersonEstimate; raise ValueError on a bad scan."""
        if not math.isfinite(stamp_s):
            raise ValueError('the scan time is not a number')
        binned = bin_scan(ranges, angle_min, angle_increment, range_min, range_max)
        if self._background is None:
            self._learn(binned)
            return PersonEstimate(False, 0, math.nan, math.nan)

        positions = self._find_people(binned)
        if len(positions) != 1:
            self._history.clear()
            return PersonEstimate(True, len(positions), math.nan, math.nan)
        position = positions[0]
        return PersonEstimate(True, 1, position, self._speed(stamp_s, position))

    def _learn(self, binned):
        self._learning.append(binned)
        if len(self._learning) < self._config.background_scans:
            return
        # The nearest thing ever seen in each direction. The two neighbouring
        # directions count as well, because a beam does not hit exactly the
        # same spot in every scan.
        nearest = np.min(self._learning, axis=0)
        self._background = np.minimum(nearest, np.minimum(np.roll(nearest, 1),
                                                          np.roll(nearest, -1)))
        self._learning = []

    def _find_people(self, binned):
        """Return the position along the path of each person, in order."""
        config = self._config
        new = binned < self._background - config.background_margin_m
        across = binned[new] * np.cos(self._angles[new])
        along = binned[new] * np.sin(self._angles[new])
        on_path = (np.abs(across - config.path_distance_m) <= config.corridor_half_width_m) \
            & (np.abs(along) <= config.zone_half_length_m)
        along = np.sort(along[on_path])
        if along.size == 0:
            return []
        # A gap wider than a long step separates two people.
        groups = np.split(along, np.flatnonzero(np.diff(along) > config.person_gap_m) + 1)
        # The middle between the outermost points is the middle between the
        # feet when they are apart, and the one visible leg when they cross.
        return [float(group[0] + group[-1]) / 2 for group in groups
                if group.size >= config.min_points]

    def _speed(self, stamp_s, position):
        """Fit a line to the recent positions; its slope is the speed."""
        config = self._config
        history = self._history
        if history:
            last_stamp, last_position = history[-1]
            if stamp_s <= last_stamp or abs(position - last_position) > config.max_jump_m:
                history.clear()  # time went back, or this is someone else
        history.append((stamp_s, position))
        while stamp_s - history[0][0] > config.speed_window_s:
            history.popleft()
        if len(history) < 3 or stamp_s - history[0][0] < config.min_speed_span_s:
            return math.nan
        times, positions = np.array(history).T
        return float(np.polyfit(times - times[0], positions, 1)[0])
