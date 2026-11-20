"""Checks of the step counter on hand-made spreads."""

from lidar_person_tracker.steps import StepCounter


def count(spreads, positions):
    counter = StepCounter(drop_m=0.12, max_spread_m=1.0, min_move_m=0.15)
    for spread, position in zip(spreads, positions):
        counter.update(spread, position)
    return counter


# The feet open and close twice: two steps.
TWO_STEPS = [0.1, 0.4, 0.6, 0.3, 0.1, 0.4, 0.6, 0.3, 0.1]


def walking(scans, step_m=0.1):
    return [index * step_m for index in range(scans)]


def test_each_opening_of_the_feet_is_one_step():
    assert count(TWO_STEPS, walking(len(TWO_STEPS))).steps == 2


def test_a_shallow_closing_still_counts():
    spreads = [0.1, 0.5, 0.55, 0.38, 0.52, 0.55, 0.3]
    assert count(spreads, walking(len(spreads))).steps == 2


def test_feet_moving_in_place_are_not_steps():
    assert count(TWO_STEPS, [0.0] * len(TWO_STEPS)).steps == 0


def test_small_wobbles_are_not_steps():
    spreads = [0.5, 0.55, 0.47, 0.56, 0.48, 0.55, 0.5]
    assert count(spreads, walking(len(spreads))).steps == 0


def test_a_spread_wider_than_two_feet_is_skipped():
    spreads = [0.1, 0.4, 0.6, 1.5, 0.6, 0.3, 0.1]
    assert count(spreads, walking(len(spreads))).steps == 1


def test_update_says_when_a_step_is_counted():
    counter = StepCounter(drop_m=0.12, max_spread_m=1.0, min_move_m=0.15)
    said = [counter.update(spread, position)
            for spread, position in zip(TWO_STEPS, walking(len(TWO_STEPS)))]
    assert said == [False, False, False, True, False, False, False, True, False]


def test_losing_the_walker_keeps_the_steps():
    counter = count(TWO_STEPS, walking(len(TWO_STEPS)))
    counter.lose_walker()
    assert counter.steps == 2
    # The next walker starts from where they are first seen.
    for spread, position in zip(TWO_STEPS, walking(len(TWO_STEPS))):
        counter.update(spread, 5.0 + position)
    assert counter.steps == 4
