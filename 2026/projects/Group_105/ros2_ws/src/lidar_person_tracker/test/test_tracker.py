"""
Checks of the tracker on made-up scans.

The scans come from a simple model: a wall behind the walking path and round
legs. They show that the logic works, not how accurate it is on a real LiDAR.
"""

import math

from lidar_person_tracker.tracker import bin_scan, PersonTracker, TrackerConfig
import numpy as np
import pytest

RANGE_MIN = 0.12
RANGE_MAX = 3.5
SCAN_PERIOD_S = 0.2
PATH_M = 2.5
WALL_M = 3.3
LEG_RADIUS_M = 0.05


def fake_scan(circles=(), beams=360, noise_m=0.0, rng=None):
    """Return the ranges a LiDAR at the origin would measure; circles are (x, y, radius)."""
    angles = 2 * math.pi * np.arange(beams) / beams
    cos, sin = np.cos(angles), np.sin(angles)
    ranges = np.full(beams, np.inf)
    facing_wall = cos > 1e-6
    ranges[facing_wall] = WALL_M / cos[facing_wall]
    for x, y, radius in circles:
        towards = cos * x + sin * y
        inside_root = towards ** 2 - (x * x + y * y - radius * radius)
        hit = (inside_root >= 0) & (towards > 0)
        ranges[hit] = np.minimum(ranges[hit], towards[hit] - np.sqrt(inside_root[hit]))
    if noise_m:
        ranges = ranges + rng.normal(0, noise_m, beams)
    ranges[ranges > RANGE_MAX] = np.inf
    return ranges, 2 * math.pi / beams


def legs(position_m, time_s=0.0, steps_per_s=1.8, across_m=PATH_M):
    """Two legs of a person at a position along the path, swinging as they walk."""
    swing = 0.3 * math.sin(math.pi * steps_per_s * time_s)
    return [
        (across_m - 0.08, position_m + swing, LEG_RADIUS_M),
        (across_m + 0.08, position_m - swing, LEG_RADIUS_M),
    ]


class Robot:
    """Feeds scans to a tracker, one every scan period."""

    def __init__(self, config=None, beams=360, noise_m=0.0):
        self.tracker = PersonTracker(config)
        self.beams = beams
        self.noise_m = noise_m
        self.rng = np.random.default_rng(1)
        self.time_s = 0.0

    def scan(self, circles=()):
        ranges, increment = fake_scan(circles, self.beams, self.noise_m, self.rng)
        estimate = self.tracker.update(
            self.time_s, ranges, 0.0, increment, RANGE_MIN, RANGE_MAX)
        self.time_s += SCAN_PERIOD_S
        return estimate

    def learn(self, circles=()):
        for _ in range(TrackerConfig().background_scans):
            estimate = self.scan(circles)
        return estimate

    def walk(self, start_m, speed_mps, duration_s):
        """Let a person walk and return the estimate of every scan."""
        start_s = self.time_s
        estimates = []
        while self.time_s - start_s < duration_s:
            elapsed_s = self.time_s - start_s
            estimates.append(self.scan(legs(start_m + speed_mps * elapsed_s, elapsed_s)))
        return estimates


def test_not_ready_while_learning():
    robot = Robot()
    assert not robot.learn().ready
    assert robot.scan().ready


def test_empty_room_has_no_person():
    robot = Robot(noise_m=0.02)
    robot.learn()
    for _ in range(20):
        estimate = robot.scan()
        assert estimate.people_count == 0
        assert not estimate.detected
        assert math.isnan(estimate.position_m)


@pytest.mark.parametrize('position_m', [-2.0, -0.5, 0.0, 1.0, 2.0])
def test_standing_person_is_found(position_m):
    robot = Robot(noise_m=0.02)
    robot.learn()
    estimate = robot.scan(legs(position_m))
    assert estimate.people_count == 1
    assert estimate.position_m == pytest.approx(position_m, abs=0.1)


def test_standing_person_has_no_speed():
    robot = Robot(noise_m=0.02)
    robot.learn()
    for _ in range(10):
        estimate = robot.scan(legs(0.5))
    assert estimate.speed_mps == pytest.approx(0.0, abs=0.1)


@pytest.mark.parametrize('speed_mps', [1.2, 0.5, -0.8])
def test_walking_speed_and_direction(speed_mps):
    robot = Robot(noise_m=0.02)
    robot.learn()
    estimates = robot.walk(start_m=-2.0 * np.sign(speed_mps), speed_mps=speed_mps,
                           duration_s=3.0)
    assert all(estimate.people_count == 1 for estimate in estimates)
    speeds = [estimate.speed_mps for estimate in estimates if not math.isnan(estimate.speed_mps)]
    assert len(speeds) >= len(estimates) - 3
    assert np.mean(speeds) == pytest.approx(speed_mps, abs=0.1)
    assert np.max(np.abs(np.array(speeds) - speed_mps)) < 0.5


def test_no_speed_from_the_first_scans():
    robot = Robot()
    robot.learn()
    estimates = robot.walk(start_m=-1.0, speed_mps=1.0, duration_s=0.4)
    assert all(math.isnan(estimate.speed_mps) for estimate in estimates)


def test_two_people_are_counted_and_not_measured():
    robot = Robot(noise_m=0.02)
    robot.learn()
    estimate = robot.scan(legs(-1.0) + legs(1.0))
    assert estimate.people_count == 2
    assert math.isnan(estimate.position_m)
    assert math.isnan(estimate.speed_mps)


def test_speed_restarts_after_two_people():
    robot = Robot()
    robot.learn()
    robot.walk(start_m=-1.0, speed_mps=1.0, duration_s=1.0)
    robot.scan(legs(0.0) + legs(1.5))
    assert math.isnan(robot.scan(legs(0.2)).speed_mps)


def test_object_present_while_learning_is_ignored():
    chair = [(PATH_M, 1.0, 0.2)]
    robot = Robot(noise_m=0.02)
    robot.learn(chair)
    assert robot.scan(chair).people_count == 0
    estimate = robot.scan(chair + legs(-1.0))
    assert estimate.people_count == 1
    assert estimate.position_m == pytest.approx(-1.0, abs=0.1)


@pytest.mark.parametrize('across_m', [0.3, 3.29])
def test_person_off_the_path_is_ignored(across_m):
    robot = Robot()
    robot.learn()
    assert robot.scan(legs(0.0, across_m=across_m)).people_count == 0


@pytest.mark.parametrize('across_m', [0.7, 1.2, 2.5, 3.0])
def test_person_is_found_at_any_distance_in_the_strip(across_m):
    robot = Robot()
    robot.learn()
    estimate = robot.scan(legs(0.5, across_m=across_m))
    assert estimate.people_count == 1
    assert estimate.position_m == pytest.approx(0.5, abs=0.1)


def test_person_beyond_the_zone_is_ignored():
    robot = Robot(TrackerConfig(zone_half_length_m=1.5))
    robot.learn()
    assert robot.scan(legs(2.0)).people_count == 0


def test_single_point_is_noise():
    robot = Robot()
    robot.learn()
    assert robot.scan([(PATH_M, 0.0, 0.01)]).people_count == 0


def test_relearn_forgets_the_room():
    robot = Robot()
    robot.learn()
    robot.tracker.relearn()
    assert not robot.scan().ready
    box = [(PATH_M, 0.0, 0.2)]
    robot.learn(box)
    assert robot.scan(box).people_count == 0


def test_other_number_of_beams():
    robot = Robot(beams=240)
    robot.learn()
    assert robot.scan().people_count == 0
    estimate = robot.scan(legs(0.5))
    assert estimate.people_count == 1
    assert estimate.position_m == pytest.approx(0.5, abs=0.15)


def test_facing_angle_turns_the_path():
    robot = Robot(TrackerConfig(facing_angle_rad=math.pi / 2))
    robot.learn()
    # With the path on the robot's left, the wall model is no longer behind it.
    estimate = robot.scan([(0.3, PATH_M, LEG_RADIUS_M), (-0.3, PATH_M, LEG_RADIUS_M)])
    assert estimate.people_count == 1
    assert estimate.position_m == pytest.approx(0.0, abs=0.1)


def test_unusable_ranges_count_as_nothing_seen():
    binned = bin_scan([0.0, math.nan, math.inf, 9.0, 1.0], 0.0, math.radians(1),
                      RANGE_MIN, RANGE_MAX)
    assert binned[4] == 1.0
    assert np.all(np.isinf(binned[:4]))


def test_bad_scans_are_rejected():
    tracker = PersonTracker()
    with pytest.raises(ValueError):
        tracker.update(0.0, [], 0.0, 0.01, RANGE_MIN, RANGE_MAX)
    with pytest.raises(ValueError):
        tracker.update(0.0, [1.0], 0.0, 0.0, RANGE_MIN, RANGE_MAX)
    with pytest.raises(ValueError):
        tracker.update(math.nan, [1.0], 0.0, 0.01, RANGE_MIN, RANGE_MAX)


@pytest.mark.parametrize('settings', [
    {'path_distance_m': 0.0},
    {'background_margin_m': -0.1},
    {'person_gap_m': math.nan},
    {'background_scans': 0},
    {'min_points': 1.5},
    {'min_speed_span_s': 2.0, 'speed_window_s': 1.0},
])
def test_bad_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        TrackerConfig(**settings)
