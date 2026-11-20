"""
Checks of the tracker on made-up scans.

The scans come from a simple model: a wall behind the walking path, round
legs, and for a driving robot some posts beside the path. They show that the
logic works, not how accurate it is on a real LiDAR.
"""

import math

from lidar_person_tracker.tracker import bin_scan, PersonTracker, TrackerConfig
import numpy as np
import pytest

RANGE_MIN = 0.12
RANGE_MAX = 3.5
SCAN_PERIOD_S = 0.11  # about 9 scans per second, like the robot's LiDAR
PATH_M = 2.5
WALL_M = 3.3
LEG_RADIUS_M = 0.05


def fake_scan(circles=(), beams=360, noise_m=0.0, rng=None, pose=(0.0, 0.0, 0.0)):
    """Return the ranges a LiDAR at pose (x, y, heading) measures; circles are (x, y, radius)."""
    robot_x, robot_y, heading = pose
    angles = 2 * math.pi * np.arange(beams) / beams
    cos, sin = np.cos(heading + angles), np.sin(heading + angles)
    ranges = np.full(beams, np.inf)
    facing_wall = cos > 1e-6
    ranges[facing_wall] = (WALL_M - robot_x) / cos[facing_wall]
    for x, y, radius in circles:
        x, y = x - robot_x, y - robot_y
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

    def __init__(self, config=None, beams=360, noise_m=0.0, furniture=(), odometry=False):
        self.tracker = PersonTracker(config)
        self.beams = beams
        self.noise_m = noise_m
        self.rng = np.random.default_rng(1)
        self.time_s = 0.0
        self.pose = (0.0, 0.0, 0.0)  # x, y, heading, in the room
        self.furniture = list(furniture)
        self.odometry = odometry  # exact odometry

    def scan(self, circles=()):
        if self.odometry:
            self.tracker.add_odometry(self.time_s, *self.pose)
        ranges, increment = fake_scan(list(circles) + self.furniture, self.beams, self.noise_m,
                                      self.rng, self.pose)
        estimate = self.tracker.update(
            self.time_s, ranges, 0.0, increment, RANGE_MIN, RANGE_MAX)
        self.time_s += SCAN_PERIOD_S
        return estimate

    def learn(self, circles=()):
        for _ in range(TrackerConfig().background_scans):
            estimate = self.scan(circles)
        return estimate

    def walk(self, start_m, speed_mps, duration_s, steps_per_s=1.8):
        """Let a person walk and return the estimate of every scan."""
        start_s = self.time_s
        estimates = []
        while self.time_s - start_s < duration_s:
            elapsed_s = self.time_s - start_s
            estimates.append(self.scan(legs(start_m + speed_mps * elapsed_s, elapsed_s,
                                            steps_per_s)))
        return estimates


def steps_taken(steps_per_s, duration_s):
    """How many times the feet of legs() are furthest apart within the duration."""
    return math.floor(steps_per_s * duration_s + 0.5)


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
    # No speed only while the first min_speed_span_s is covered.
    no_speed_scans = math.ceil(TrackerConfig().min_speed_span_s / SCAN_PERIOD_S)
    assert len(speeds) >= len(estimates) - no_speed_scans
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


@pytest.mark.parametrize('steps_per_s', [1.4, 1.8, 2.2])
def test_steps_are_counted_while_walking(steps_per_s):
    robot = Robot(noise_m=0.02)
    robot.learn()
    estimates = robot.walk(start_m=-1.5, speed_mps=1.0, duration_s=3.0, steps_per_s=steps_per_s)
    assert estimates[-1].steps == pytest.approx(steps_taken(steps_per_s, 3.0), abs=1)


def test_standing_with_the_feet_apart_counts_no_steps():
    robot = Robot(noise_m=0.02)
    robot.learn()
    for _ in range(50):
        # The weight shifts from foot to foot, so the feet seem to move a little.
        half_m = 0.25 + 0.08 * math.sin(4 * robot.time_s)
        estimate = robot.scan([(PATH_M - 0.08, 0.5 + half_m, LEG_RADIUS_M),
                               (PATH_M + 0.08, 0.5 - half_m, LEG_RADIUS_M)])
        assert estimate.spread_m > 0.3
    assert estimate.steps == 0


def test_steps_are_kept_when_the_walker_is_lost():
    robot = Robot()
    robot.learn()
    before = robot.walk(start_m=-1.5, speed_mps=1.0, duration_s=1.5)[-1].steps
    assert before > 0
    assert robot.scan(legs(0.0) + legs(1.5)).steps == before
    assert robot.scan().steps == before
    after = robot.walk(start_m=0.0, speed_mps=1.0, duration_s=1.5)[-1].steps
    assert after > before


def test_relearn_starts_the_steps_again():
    robot = Robot()
    robot.learn()
    assert robot.walk(start_m=-1.5, speed_mps=1.0, duration_s=1.5)[-1].steps > 0
    robot.tracker.relearn()
    robot.learn()
    assert robot.scan().steps == 0


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
    {'step_drop_m': 0.0},
    {'step_drop_m': 1.0, 'step_max_spread_m': 1.0},
])
def test_bad_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        TrackerConfig(**settings)


# Driving mode. The robot drives along the walking path, which runs along the
# room's y axis at x = PATH_M. It heads along the path, and the path is on
# its right, where the camera points.

RIGHT_RAD = -math.pi / 2
POSTS = [(3.22, 1.0, 0.05), (3.22, -2.5, 0.05), (3.22, 3.5, 0.05)]  # behind the path


def driving_robot(mode='driving'):
    robot = Robot(TrackerConfig(facing_angle_rad=RIGHT_RAD), noise_m=0.02, furniture=POSTS,
                  odometry=True)
    robot.tracker.set_mode(mode)
    robot.pose = (0.0, 0.0, math.pi / 2)
    robot.learn()
    return robot


def walk_beside(robot, start_m, speed_mps, duration_s, robot_speed_mps=None, walker=True):
    """Let the walker walk along the path while the robot drives beside it (by default as fast)."""
    robot_speed_mps = speed_mps if robot_speed_mps is None else robot_speed_mps
    x_m, y_m, heading = robot.pose
    start_s = robot.time_s
    estimates = []
    while robot.time_s - start_s < duration_s:
        elapsed_s = robot.time_s - start_s
        robot.pose = (x_m, y_m + robot_speed_mps * elapsed_s, heading)
        circles = legs(start_m + speed_mps * elapsed_s, elapsed_s) if walker else []
        estimates.append(robot.scan(circles))
    return estimates


def known_speeds(estimates):
    return [estimate.speed_mps for estimate in estimates if not math.isnan(estimate.speed_mps)]


def followed(estimates):
    """Check that the walker was seen and followed in every scan; return the scans."""
    assert all(estimate.following and estimate.people_count == 1 for estimate in estimates)
    return estimates


def test_static_mode_stops_using_the_room_once_the_robot_drives():
    robot = driving_robot(mode='static')
    estimates = walk_beside(robot, start_m=-1.0, speed_mps=0.8, duration_s=2.0)
    assert estimates[0].people_count == 1
    # Otherwise everything that moves in the scan as the robot drives would look like a person.
    assert all(estimate.people_count == 0 and estimate.moved for estimate in estimates[5:])


@pytest.mark.parametrize('speed_mps', [0.8, -0.8])
def test_driving_mode_follows_the_walker_forward_and_backward(speed_mps):
    robot = driving_robot()
    estimates = walk_beside(robot, start_m=-1.0, speed_mps=speed_mps, duration_s=4.0)
    seen = followed(estimates)
    # From the robot, the walker stays where they were; over the floor they walk.
    assert all(abs(estimate.position_m + 1.0) < 0.15 for estimate in seen)
    assert np.mean(known_speeds(seen)) == pytest.approx(speed_mps, abs=0.1)
    assert estimates[-1].steps == pytest.approx(steps_taken(1.8, 4.0), abs=1)


def test_driving_mode_reports_the_gap_when_the_robot_is_slower():
    robot = driving_robot()
    estimates = walk_beside(robot, start_m=0.5, speed_mps=0.8, duration_s=3.0,
                            robot_speed_mps=0.4)
    seen = followed(estimates)
    assert seen[-1].position_m == pytest.approx(0.5 + 0.4 * 3.0, abs=0.2)
    assert np.mean(known_speeds(seen)[-10:]) == pytest.approx(0.8, abs=0.1)


def test_a_short_gap_keeps_following():
    robot = driving_robot()
    walk_beside(robot, start_m=-1.0, speed_mps=0.8, duration_s=2.0)
    gap = walk_beside(robot, start_m=0.6, speed_mps=0.8, duration_s=0.3, walker=False)
    assert all(estimate.following and estimate.people_count == 0 for estimate in gap)
    after = followed(walk_beside(robot, start_m=0.84, speed_mps=0.8, duration_s=1.0))
    assert not math.isnan(after[0].speed_mps)


def test_a_lost_walker_is_not_found_again_once_the_robot_has_moved():
    robot = driving_robot()
    walk_beside(robot, start_m=-1.0, speed_mps=0.8, duration_s=2.0)
    gone = walk_beside(robot, start_m=0.6, speed_mps=0.8, duration_s=1.5, walker=False)
    assert not gone[-1].following
    assert gone[-1].moved
    back = walk_beside(robot, start_m=1.8, speed_mps=0.8, duration_s=1.0)
    assert all(estimate.people_count == 0 and estimate.moved for estimate in back)
    robot.tracker.relearn()
    robot.learn()
    assert robot.scan(legs(robot.pose[1] + 1.0)).people_count == 1


def test_a_parked_robot_in_driving_mode_finds_the_walker_again():
    robot = driving_robot()
    walk_beside(robot, start_m=-1.5, speed_mps=0.8, duration_s=2.0, robot_speed_mps=0.0)
    gone = walk_beside(robot, start_m=0.1, speed_mps=0.8, duration_s=1.5, robot_speed_mps=0.0,
                       walker=False)
    assert not gone[-1].following and not gone[-1].moved
    back = walk_beside(robot, start_m=1.0, speed_mps=-0.8, duration_s=1.0, robot_speed_mps=0.0)
    assert all(estimate.people_count == 1 and estimate.following for estimate in back)


def test_the_walker_must_be_found_before_the_robot_drives():
    robot = driving_robot()
    walk_beside(robot, start_m=0.0, speed_mps=0.8, duration_s=1.0, walker=False)
    estimates = walk_beside(robot, start_m=0.8, speed_mps=0.8, duration_s=1.0)
    assert all(estimate.people_count == 0 and estimate.moved for estimate in estimates)


def test_switching_modes():
    robot = driving_robot(mode='static')
    assert not robot.scan(legs(0.5)).following
    robot.tracker.set_mode('driving')
    assert robot.scan(legs(0.5)).following
    robot.tracker.set_mode('static')
    estimate = robot.scan(legs(0.5))
    assert estimate.people_count == 1
    assert not estimate.following


def test_late_odometry_does_not_stop_the_static_mode():
    robot = driving_robot(mode='static')
    assert robot.scan(legs(0.5)).people_count == 1
    robot.odometry = False  # the robot's last pose is still where it learned the room
    for _ in range(5):
        estimate = robot.scan(legs(0.5))
    assert estimate.people_count == 1
    assert not estimate.moved


@pytest.mark.parametrize('turn_rad, moved', [(0.05, False), (0.2, True)])
def test_a_small_drift_of_the_heading_is_not_a_move(turn_rad, moved):
    robot = driving_robot(mode='static')
    robot.odometry = False
    robot.tracker.add_odometry(robot.time_s, 0.0, 0.0, math.pi / 2 + turn_rad)
    assert robot.scan(legs(0.5)).moved == moved


def test_bad_mode_and_odometry_are_rejected():
    tracker = PersonTracker()
    with pytest.raises(ValueError):
        tracker.set_mode('flying')
    with pytest.raises(ValueError):
        tracker.add_odometry(0.0, math.nan, 0.0, 0.0)
