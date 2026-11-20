"""
Find a walking person in the LiDAR scans, with the robot parked or driving.

Static mode, robot parked: the empty room always gives the same scan. The
tracker first remembers that scan. After that, a point that is clearly nearer
than the empty room, and lies on the walking path, belongs to a person.

Driving mode: the walker is first found in the same way, while the robot is
still parked. From then on the tracker follows the walker's points: in each
scan it only looks near where the walker should be. The odometry places each
scan in a fixed frame, so the robot's own movement is taken out.

In both modes the empty room is only used while the robot stands where it
learned it; once the odometry shows that it has moved, the room must be
learned again. The steps are counted from the walker's points (see steps.py).

This module has no ROS imports, so it can be tested without ROS.
"""

from collections import deque
from dataclasses import dataclass
import math

from lidar_person_tracker.odometry import Pose, PoseHistory
from lidar_person_tracker.steps import StepCounter
import numpy as np

BINS = 360  # one direction per degree, all around the robot
MODES = ('static', 'driving')


@dataclass(frozen=True)
class TrackerConfig:
    """Settings of the tracker; lengths in metres, times in seconds."""

    # Watch the floor from 0.5 m to 3.5 m in front of the robot, so that the
    # exact distance of the walking line does not have to be set for each room.
    path_distance_m: float = 2.0
    corridor_half_width_m: float = 1.5
    zone_half_length_m: float = 2.5
    facing_angle_rad: float = 0.0
    background_scans: int = 15
    background_margin_m: float = 0.2
    person_gap_m: float = 0.8
    min_points: int = 2
    speed_window_s: float = 1.0
    min_speed_span_s: float = 0.4
    max_jump_m: float = 1.0
    # A step: the walker's feet close up by step_drop_m after being apart,
    # and the walker has moved on by step_min_move_m since the feet were last
    # apart. A spread wider than step_max_spread_m is not two feet.
    step_drop_m: float = 0.12
    step_min_move_m: float = 0.15
    step_max_spread_m: float = 1.0
    # Driving mode: the walker's points are looked for within follow_radius_m
    # of where the walker should be; none for follow_lost_s and the walker
    # is lost.
    follow_radius_m: float = 0.6
    follow_lost_s: float = 1.0
    # The empty room is only used while the robot is within parked_m and
    # parked_rad of where it learned it. The heading in the odometry can
    # drift a little while the robot stands still, so the turn limit is wide.
    parked_m: float = 0.05
    parked_rad: float = 0.1

    def __post_init__(self):
        positive = [
            'path_distance_m', 'corridor_half_width_m', 'zone_half_length_m',
            'background_margin_m', 'person_gap_m', 'speed_window_s',
            'min_speed_span_s', 'max_jump_m', 'step_drop_m', 'step_min_move_m',
            'step_max_spread_m', 'follow_radius_m', 'follow_lost_s', 'parked_m', 'parked_rad',
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
        if self.step_drop_m >= self.step_max_spread_m:
            raise ValueError('step_drop_m must be smaller than step_max_spread_m')


@dataclass(frozen=True)
class PersonEstimate:
    """What one scan says about people on the walking path."""

    ready: bool  # False while the empty room is still being learned
    people_count: int
    position_m: float  # along the path, from the robot; NaN unless exactly one person
    speed_mps: float  # along the path, over the floor; NaN until enough scans of one person
    steps: int  # counted since the room was learned
    spread_m: float  # along the path, between the person's outermost points; NaN unless one person
    following: bool = False  # driving mode: following the walker's points
    moved: bool = False  # the robot moved since it learned the room, which is then not used

    @property
    def detected(self):
        return self.people_count > 0


@dataclass(frozen=True)
class _Person:
    """One group of points on the path."""

    position_m: float  # along the path, middle between the outermost points
    spread_m: float  # along the path, between the outermost points
    across_m: float  # across the path, the median of the points


class _Walker:
    """The walker followed in driving mode, in the odom frame."""

    def __init__(self, stamp_s, pose, person, facing_angle_rad):
        # The path's directions stay fixed in the odom frame from now on.
        facing = pose.yaw_rad + facing_angle_rad
        self.across_axis = np.array([math.cos(facing), math.sin(facing)])
        self.along_axis = np.array([-math.sin(facing), math.cos(facing)])
        self.position = np.array([pose.x_m, pose.y_m]) + self.across_axis * person.across_m \
            + self.along_axis * person.position_m
        self.seen_s = stamp_s
        self.speed_mps = 0.0

    def along_from_robot(self, pose):
        """Return how far the odom frame's along-path values are from the robot's."""
        return float(np.dot([pose.x_m, pose.y_m], self.along_axis))


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
        self._poses = PoseHistory()
        self._mode = 'static'
        self.relearn()

    @property
    def mode(self):
        return self._mode

    def set_mode(self, mode):
        """Choose 'static' (robot parked) or 'driving'; raise ValueError on another mode."""
        if mode not in MODES:
            raise ValueError(f'mode must be one of {", ".join(MODES)}, got {mode!r}')
        self._mode = mode
        if mode == 'static' and self._walker is not None:
            self._lose_walker()

    def relearn(self):
        """Forget the empty room and learn it again from the next scans."""
        config = self._config
        self._background = None
        self._learning = []
        self._learned_pose = None
        self._walker = None
        self._history.clear()
        self._steps = StepCounter(config.step_drop_m, config.step_max_spread_m,
                                  config.step_min_move_m)

    def add_odometry(self, stamp_s, x_m, y_m, yaw_rad):
        """Take where the robot is in the odom frame; raise ValueError on a bad value."""
        if not all(math.isfinite(value) for value in (stamp_s, x_m, y_m, yaw_rad)):
            raise ValueError('the odometry is not a number')
        self._poses.add(stamp_s, Pose(x_m, y_m, yaw_rad))

    def update(self, stamp_s, ranges, angle_min, angle_increment, range_min, range_max):
        """Take one scan and return a PersonEstimate; raise ValueError on a bad scan."""
        if not math.isfinite(stamp_s):
            raise ValueError('the scan time is not a number')
        binned = bin_scan(ranges, angle_min, angle_increment, range_min, range_max)
        pose = self._poses.at(stamp_s)  # where the robot was at the scan; None if unknown
        if self._background is None:
            self._learn(binned)
            return PersonEstimate(False, 0, math.nan, math.nan, 0, math.nan)
        if self._walker is not None:
            return self._follow(stamp_s, binned, pose)
        if self._moved():
            return self._nobody(0, moved=True)

        people = self._find_people(binned)
        if len(people) != 1:
            return self._nobody(len(people))
        person = people[0]
        speed = self._speed(stamp_s, person.position_m)
        self._steps.update(person.spread_m, person.position_m)
        following = self._mode == 'driving' and pose is not None
        if following:
            self._start_following(stamp_s, pose, person)
        return PersonEstimate(True, 1, person.position_m, speed, self._steps.steps,
                              person.spread_m, following)

    def _nobody(self, people_count, following=False, moved=False):
        """Return an estimate without a position, and forget the walker's movement."""
        self._history.clear()
        self._steps.lose_walker()
        return PersonEstimate(True, people_count, math.nan, math.nan, self._steps.steps,
                              math.nan, following, moved)

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
        self._learned_pose = self._poses.latest()

    def _moved(self):
        """Return True when the robot is no longer where it learned the empty room."""
        # The last pose the odometry gave, so a message that comes late over
        # Wi-Fi does not matter. Without any odometry, the robot is taken to
        # be parked.
        pose = self._poses.latest()
        if pose is None:
            return False
        if self._learned_pose is None:
            self._learned_pose = pose  # the odometry started after the room was learned
            return False
        distance, turn = pose.moved_from(self._learned_pose)
        return distance > self._config.parked_m or turn > self._config.parked_rad

    def _find_people(self, binned):
        """Return a _Person for each group of new points on the path, in order along it."""
        config = self._config
        new = binned < self._background - config.background_margin_m
        across = binned[new] * np.cos(self._angles[new])
        along = binned[new] * np.sin(self._angles[new])
        on_path = (np.abs(across - config.path_distance_m) <= config.corridor_half_width_m) \
            & (np.abs(along) <= config.zone_half_length_m)
        return self._group(along[on_path], across[on_path], config.person_gap_m)

    def _group(self, along, across, gap_m):
        """Split points into people where they are further apart along the path than gap_m."""
        order = np.argsort(along)
        along, across = along[order], across[order]
        if along.size == 0:
            return []
        splits = np.flatnonzero(np.diff(along) > gap_m) + 1
        # The middle between the outermost points is the middle between the
        # feet when they are apart, and the one visible leg when they cross.
        return [_Person(float(group[0] + group[-1]) / 2, float(group[-1] - group[0]),
                        float(np.median(group_across)))
                for group, group_across in zip(np.split(along, splits), np.split(across, splits))
                if group.size >= self._config.min_points]

    def _start_following(self, stamp_s, pose, person):
        walker = _Walker(stamp_s, pose, person, self._config.facing_angle_rad)
        self._walker = walker
        # From now on the positions behind the speed are measured in the odom
        # frame, not from the robot.
        offset = walker.along_from_robot(pose)
        self._history = deque((time_s, position + offset) for time_s, position in self._history)
        self._steps.lose_walker()

    def _lose_walker(self):
        self._walker = None
        self._history.clear()
        self._steps.lose_walker()

    def _follow(self, stamp_s, binned, pose):
        """Find the walker's points near where the walker should be, in the odom frame."""
        config = self._config
        walker = self._walker
        person = self._find_walker(stamp_s, binned, pose) if pose is not None else None
        if person is None:
            if stamp_s - walker.seen_s > config.follow_lost_s:
                self._walker = None
                return self._nobody(0, moved=self._moved())
            # Not seen for a moment, or no odometry for this scan: keep the
            # speed and the steps going.
            return PersonEstimate(True, 0, math.nan, math.nan, self._steps.steps, math.nan,
                                  following=True)

        walker.position = walker.across_axis * person.across_m \
            + walker.along_axis * person.position_m
        walker.seen_s = stamp_s
        speed = self._speed(stamp_s, person.position_m)
        walker.speed_mps = speed if math.isfinite(speed) else 0.0
        self._steps.update(person.spread_m, person.position_m)
        return PersonEstimate(True, 1, person.position_m - walker.along_from_robot(pose), speed,
                              self._steps.steps, person.spread_m, following=True)

    def _find_walker(self, stamp_s, binned, pose):
        """Return the walker as a _Person in odom-frame values, or None if no points are near."""
        walker = self._walker
        expected = walker.position + walker.along_axis * walker.speed_mps \
            * (stamp_s - walker.seen_s)
        seen = np.isfinite(binned)
        directions = pose.yaw_rad + np.radians(np.arange(BINS))[seen]
        points = np.column_stack((pose.x_m + binned[seen] * np.cos(directions),
                                  pose.y_m + binned[seen] * np.sin(directions)))
        near = points[np.hypot(*(points - expected).T) <= self._config.follow_radius_m]
        # Everything near the walker is taken to be the walker.
        people = self._group(near @ walker.along_axis, near @ walker.across_axis, math.inf)
        return people[0] if people else None

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
