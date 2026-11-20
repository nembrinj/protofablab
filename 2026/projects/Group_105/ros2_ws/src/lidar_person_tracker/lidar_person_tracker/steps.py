"""
Count the walker's steps from how far apart their feet are, scan after scan.

The spread is the distance along the path between the walker's two outermost
points. It is widest when the feet are furthest apart, as a foot lands, and
narrowest when one leg passes the other. So each step is one wide moment
between two narrow ones.

A step is counted when the spread has opened up and then closes again by at
least a set amount. To be a step, the walker must also have moved on along the
path since the last time the feet were apart: someone standing with the feet
apart, shifting their weight, makes the spread wobble too, but stays in place.

This module has no ROS imports, so it can be tested without ROS.
"""


class StepCounter:
    """Counts the steps of one walker; lengths in metres."""

    def __init__(self, drop_m, max_spread_m, min_move_m):
        self._drop_m = drop_m
        self._max_spread_m = max_spread_m
        self._min_move_m = min_move_m
        self.steps = 0
        self.lose_walker()

    def lose_walker(self):
        """Forget where the walker was; the steps counted so far stay."""
        self._opening = True
        self._widest = None  # widest spread since the feet last closed up
        self._widest_position = None
        self._narrowest = None  # narrowest spread since the feet last opened up
        self._last_wide_position = None

    def update(self, spread_m, position_m):
        """Take the spread and position of one scan; return True when a step is counted."""
        if spread_m > self._max_spread_m:
            return False  # not two feet: something else is right next to the walker
        if self._widest is None:
            self._widest = self._narrowest = spread_m
            self._widest_position = self._last_wide_position = position_m
            return False
        if not self._opening:
            if spread_m < self._narrowest:
                self._narrowest = spread_m
            elif spread_m > self._narrowest + self._drop_m:
                self._opening = True
                self._widest, self._widest_position = spread_m, position_m
            return False
        if spread_m > self._widest:
            self._widest, self._widest_position = spread_m, position_m
            return False
        if spread_m >= self._widest - self._drop_m:
            return False
        # The feet were apart and are now closing up.
        moved_m = abs(self._widest_position - self._last_wide_position)
        self._last_wide_position = self._widest_position
        self._opening = False
        self._narrowest = spread_m
        if moved_m < self._min_move_m:
            return False
        self.steps += 1
        return True
