"""
Follow a walking session from what the LiDAR tracker says about the walker.

A walk is one way along the path. With the robot parked, the LiDAR sees only
the part of the path in front of it, so a walk is the part of one way that it
saw.

- Walk starts: one person moves faster than walk_speed_mps the same way for
  walk_start_s.
- Walk ends: the person moves the other way for walk_start_s; the walk ends
  where they turned, the furthest point they reached. Or the person is not
  seen for lost_s, because they left the zone.
- Stop: during a walk, slower than stop_speed_mps, and no new step, for stop_s.
- Not measured: two people were seen during the walk, or the walker was lost
  in the middle of the zone.

Steps per minute and speed are over the time the walker moved (faster than
stop_speed_mps), so stops, pauses and turning on the spot do not lower them.

The times come from the scans, so a recording can be replayed at any speed.
This module has no ROS imports, so it can be tested without ROS.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SessionConfig:
    """Settings of the session; lengths in metres, times in seconds."""

    walk_speed_mps: float = 0.3
    walk_start_s: float = 1.0
    stop_speed_mps: float = 0.15
    stop_s: float = 3.0
    lost_s: float = 2.0
    zone_edge_m: float = 1.8

    def __post_init__(self):
        for name in ['walk_speed_mps', 'walk_start_s', 'stop_speed_mps', 'stop_s', 'lost_s',
                     'zone_edge_m']:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a positive number, got {value}')
        if self.stop_speed_mps > self.walk_speed_mps:
            raise ValueError('stop_speed_mps must not be faster than walk_speed_mps')


@dataclass(frozen=True)
class Seen:
    """What the tracker said in one message."""

    stamp_s: float
    people_count: int
    position_m: float
    speed_mps: float
    steps: int


@dataclass(frozen=True)
class Walk:
    """The numbers of one walk that has ended."""

    number: int
    measured: bool
    duration_s: float  # from start to end, stops included
    moving_s: float  # the part of it the walker moved
    steps: int
    distance_m: float  # along the path, from where the walk started to where it ended
    stops: int

    @property
    def cadence_spm(self):
        """Return the steps per minute while moving, or NaN when unknown."""
        if self.moving_s <= 0:
            return math.nan
        return self.steps / self.moving_s * 60

    @property
    def speed_mps(self):
        """Return the distance per second while moving, or NaN when unknown."""
        if self.moving_s <= 0:
            return math.nan
        return abs(self.distance_m) / self.moving_s


@dataclass(frozen=True)
class Update:
    """What happened with one message."""

    started: int = 0  # number of the walk that has just started, 0 if none
    stopped: bool = False  # the walker has just stopped
    ended: Walk = None  # the walk that has just ended


@dataclass(frozen=True)
class _Mark:
    """A moment of the session: when, where, and the steps and moving time so far."""

    stamp_s: float
    position_m: float
    steps: int
    moving_s: float


class _OpenWalk:
    """A walk that has not ended yet."""

    def __init__(self, number, direction, start):
        self.number = number
        self.direction = direction
        self.start = start
        self.furthest = start
        self.measured = True
        self.stops = 0
        self.slow_since = None  # _Mark: slow and no new step since then
        self.stopped = False


class WalkSession:
    """Turns the tracker's messages into walks and stops."""

    def __init__(self, config=None):
        self._config = config or SessionConfig()
        self.walks = 0
        self._last_stamp_s = None
        self._tracker_steps = None
        self._steps = 0  # since the session started
        self._moving_s = 0.0  # since the session started
        self._moving = None  # (direction, _Mark of when it started moving that way)
        self._last_seen = None  # _Mark of the last message with one person
        self._walk = None

    @property
    def walk_in_progress(self):
        """Return the number of the walk that has not ended yet, or 0."""
        return self._walk.number if self._walk else 0

    def forget_walk(self):
        """Drop the walk in progress, for example when the time went back."""
        self._walk = None
        self._moving = None
        self._last_seen = None
        self._last_stamp_s = None

    def update(self, seen):
        """Take one tracker message and return an Update; raise ValueError on a bad one."""
        if not math.isfinite(seen.stamp_s):
            raise ValueError('the message time is not a number')
        last_stamp_s = self._last_stamp_s
        if last_stamp_s is not None and seen.stamp_s < last_stamp_s:
            raise ValueError('the time went back')
        self._last_stamp_s = seen.stamp_s
        self._count_steps(seen.steps)
        if seen.people_count != 1:
            return self._not_one_person(seen)

        if last_stamp_s is not None and math.isfinite(seen.speed_mps) \
                and abs(seen.speed_mps) >= self._config.stop_speed_mps:
            # A long gap between messages is not counted as moving.
            self._moving_s += min(seen.stamp_s - last_stamp_s, self._config.lost_s)
        now = _Mark(seen.stamp_s, seen.position_m, self._steps, self._moving_s)
        self._last_seen = now
        self._follow_movement(seen.speed_mps, now)

        walk = self._walk
        if walk is None:
            return self._maybe_start()
        if self._turned():
            ended = self._end(walk.furthest)
            self.walks += 1
            self._walk = _OpenWalk(self.walks, -walk.direction, walk.furthest)
            return Update(started=self.walks, ended=ended)
        if walk.direction * (now.position_m - walk.furthest.position_m) > 0:
            walk.furthest = now
        return Update(stopped=self._follow_stops(seen.speed_mps, now))

    def _count_steps(self, tracker_steps):
        if self._tracker_steps is not None:
            new = tracker_steps - self._tracker_steps
            # A smaller count means the tracker started counting again.
            self._steps += new if new >= 0 else tracker_steps
        self._tracker_steps = tracker_steps

    def _not_one_person(self, seen):
        walk = self._walk
        self._moving = None
        if walk is None:
            return Update()
        if seen.people_count > 1:
            walk.measured = False
        last = self._last_seen
        if last is None or seen.stamp_s - last.stamp_s < self._config.lost_s:
            return Update()
        # Lost. At the edge of the zone the walker has left it; in the middle
        # the walker was lost, and the walk was not measured to its end.
        if abs(last.position_m) < self._config.zone_edge_m:
            walk.measured = False
        return Update(ended=self._end(last))

    def _follow_movement(self, speed_mps, now):
        if not math.isfinite(speed_mps) or abs(speed_mps) <= self._config.walk_speed_mps:
            self._moving = None
            return
        direction = 1 if speed_mps > 0 else -1
        if self._moving is None or self._moving[0] != direction:
            self._moving = (direction, now)

    def _moving_long_enough(self):
        moving = self._moving
        return moving is not None \
            and self._last_seen.stamp_s - moving[1].stamp_s >= self._config.walk_start_s

    def _maybe_start(self):
        if not self._moving_long_enough():
            return Update()
        self.walks += 1
        self._walk = _OpenWalk(self.walks, self._moving[0], self._moving[1])
        return Update(started=self.walks)

    def _turned(self):
        return self._moving_long_enough() and self._moving[0] == -self._walk.direction

    def _follow_stops(self, speed_mps, now):
        """Return True when the walker has just stopped."""
        walk = self._walk
        slow = math.isfinite(speed_mps) and abs(speed_mps) < self._config.stop_speed_mps
        if not slow or (walk.slow_since is not None and now.steps != walk.slow_since.steps):
            walk.slow_since = None
            walk.stopped = False
            if not slow:
                return False
        if walk.slow_since is None:
            walk.slow_since = now
        if walk.stopped or now.stamp_s - walk.slow_since.stamp_s < self._config.stop_s:
            return False
        walk.stopped = True
        walk.stops += 1
        return True

    def _end(self, end):
        walk = self._walk
        self._walk = None
        return Walk(
            number=walk.number,
            measured=walk.measured,
            duration_s=end.stamp_s - walk.start.stamp_s,
            moving_s=end.moving_s - walk.start.moving_s,
            steps=end.steps - walk.start.steps,
            distance_m=end.position_m - walk.start.position_m,
            stops=walk.stops,
        )
