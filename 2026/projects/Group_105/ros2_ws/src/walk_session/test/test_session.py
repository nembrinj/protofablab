"""Checks of the session logic on made-up tracker messages."""

import math

import pytest
from walk_session.session import Seen, SessionConfig, WalkSession

PERIOD_S = 0.11  # the tracker sends one message per scan, about 9 per second
STEPS_PER_S = 1.8


class Walker:
    """Sends what the tracker would say about one person, one message per scan."""

    def __init__(self, position_m=0.0, config=None):
        self.session = WalkSession(config)
        self.time_s = 0.0
        self.position_m = position_m
        self.steps = 0.0
        self.updates = []

    def walk(self, speed_mps, duration_s, steps_per_s=STEPS_PER_S):
        for _ in range(round(duration_s / PERIOD_S)):
            self.position_m += speed_mps * PERIOD_S
            self.steps += steps_per_s * PERIOD_S
            self.send(1, speed_mps)

    def stand(self, duration_s):
        self.walk(0.0, duration_s, steps_per_s=0.0)

    def away(self, duration_s, people=0):
        for _ in range(round(duration_s / PERIOD_S)):
            self.send(people, math.nan)

    def send(self, people, speed_mps):
        self.time_s += PERIOD_S
        position_m = self.position_m if people == 1 else math.nan
        self.updates.append(self.session.update(
            Seen(self.time_s, people, position_m, speed_mps, math.floor(self.steps))))

    @property
    def walks(self):
        return [update.ended for update in self.updates if update.ended]

    @property
    def stops(self):
        return [index for index, update in enumerate(self.updates) if update.stopped]


def test_a_walk_through_the_zone():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 5.5)
    walker.away(3.0)
    [walk] = walker.walks
    assert walk.number == 1
    assert walk.measured
    assert walk.duration_s == pytest.approx(5.5, abs=0.2)
    assert walk.steps == pytest.approx(STEPS_PER_S * 5.5, abs=1)
    assert walk.cadence_spm == pytest.approx(STEPS_PER_S * 60, rel=0.1)
    assert walk.speed_mps == pytest.approx(0.8, abs=0.05)
    assert walk.stops == 0


def test_the_walk_starts_after_moving_one_way_for_a_while():
    walker = Walker()
    walker.walk(0.8, 0.8)
    assert not any(update.started for update in walker.updates)
    walker.walk(0.8, 0.4)
    assert [update.started for update in walker.updates if update.started] == [1]


def test_a_short_movement_is_not_a_walk():
    walker = Walker()
    walker.walk(0.8, 0.5)
    walker.stand(3.0)
    walker.away(3.0)
    assert walker.walks == []
    assert walker.session.walks == 0


def test_turning_ends_the_walk_and_starts_the_next():
    walker = Walker(position_m=-0.2)
    walker.walk(0.8, 3.0)
    walker.walk(-0.8, 5.5)
    walker.away(3.0)
    first, second = walker.walks
    assert (first.number, second.number) == (1, 2)
    # The walks meet where the walker turned.
    assert first.distance_m == pytest.approx(2.4, abs=0.15)
    assert second.distance_m == pytest.approx(-4.4, abs=0.15)
    assert first.duration_s == pytest.approx(3.0, abs=0.2)
    assert first.measured and second.measured
    assert first.steps + second.steps == pytest.approx(STEPS_PER_S * 8.5, abs=1)


def test_turning_on_the_spot_is_not_walking_time():
    walker = Walker(position_m=-0.2)
    walker.walk(0.8, 3.0)
    walker.stand(1.0)
    walker.walk(-0.8, 5.5)
    walker.away(3.0)
    first, second = walker.walks
    assert first.moving_s == pytest.approx(3.0, abs=0.2)
    assert second.duration_s == pytest.approx(6.5, abs=0.2)
    assert second.moving_s == pytest.approx(5.5, abs=0.2)
    assert second.cadence_spm == pytest.approx(STEPS_PER_S * 60, rel=0.1)
    assert second.speed_mps == pytest.approx(0.8, abs=0.05)
    assert first.stops == second.stops == 0


def test_a_stop_is_noticed_once_and_left_out_of_the_walking_time():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.0)
    stand_from = len(walker.updates)
    walker.stand(5.0)
    walker.walk(0.8, 3.5)
    walker.away(3.0)
    [stop] = walker.stops
    assert (stop - stand_from) * PERIOD_S == pytest.approx(3.0, abs=0.2)
    [walk] = walker.walks
    assert walk.stops == 1
    assert walk.duration_s == pytest.approx(10.5, abs=0.2)
    assert walk.moving_s == pytest.approx(5.5, abs=0.2)
    assert walk.cadence_spm == pytest.approx(STEPS_PER_S * 60, rel=0.1)
    assert walk.speed_mps == pytest.approx(0.8, abs=0.05)


def test_the_walk_in_progress_so_far():
    walker = Walker(position_m=-2.2)
    assert walker.session.progress is None
    walker.walk(0.8, 2.0)
    progress = walker.session.progress
    assert progress.number == 1
    assert progress.steps == pytest.approx(STEPS_PER_S * 2.0, abs=1)
    assert (progress.stops, progress.stopped) == (0, False)
    walker.stand(4.0)
    assert (walker.session.progress.stops, walker.session.progress.stopped) == (1, True)
    walker.walk(0.8, 1.0)
    assert (walker.session.progress.stops, walker.session.progress.stopped) == (1, False)
    walker.away(3.0)
    assert walker.session.progress is None


def test_a_short_pause_is_not_a_stop():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.0)
    walker.stand(2.0)
    walker.walk(0.8, 3.5)
    walker.away(3.0)
    assert walker.stops == []
    assert walker.walks[0].stops == 0


def test_slow_steps_are_not_a_stop():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.0)
    walker.walk(0.1, 5.0)
    assert walker.stops == []


def test_a_second_person_marks_the_walk_not_measured():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.5)
    walker.away(0.5, people=2)
    walker.walk(0.8, 3.0)
    walker.away(3.0)
    [walk] = walker.walks
    assert not walk.measured


def test_a_walker_lost_in_the_middle_is_not_measured():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.5)
    walker.away(3.0)
    [walk] = walker.walks
    assert not walk.measured
    assert walk.duration_s == pytest.approx(2.5, abs=0.2)


def test_a_short_loss_does_not_end_the_walk():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.5)
    walker.away(1.0)
    walker.walk(0.8, 3.0)
    walker.away(3.0)
    [walk] = walker.walks
    assert walk.measured


def test_steps_go_on_when_the_tracker_counts_again_from_zero():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.5)
    walker.steps = 0.0  # the tracker learned the empty room again
    walker.walk(0.8, 3.0)
    walker.away(3.0)
    [walk] = walker.walks
    assert walk.steps == pytest.approx(STEPS_PER_S * 5.5, abs=2)


def test_time_going_back_is_rejected():
    session = WalkSession()
    session.update(Seen(10.0, 0, math.nan, math.nan, 0))
    with pytest.raises(ValueError):
        session.update(Seen(9.0, 0, math.nan, math.nan, 0))
    with pytest.raises(ValueError):
        session.update(Seen(math.nan, 0, math.nan, math.nan, 0))


def test_forget_walk_drops_the_walk_in_progress():
    walker = Walker(position_m=-2.2)
    walker.walk(0.8, 2.5)
    assert walker.session.walk_in_progress == 1
    walker.session.forget_walk()
    assert walker.session.walk_in_progress == 0
    walker.away(3.0)
    assert walker.walks == []


@pytest.mark.parametrize('settings', [
    {'stop_s': 0.0},
    {'lost_s': math.nan},
    {'walk_speed_mps': -0.3},
    {'stop_speed_mps': 0.5, 'walk_speed_mps': 0.3},
])
def test_bad_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        SessionConfig(**settings)
