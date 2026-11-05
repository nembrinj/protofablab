"""Regression checks for counting behavior; these do not establish real-video accuracy."""
import unittest

import numpy as np

from walk_analysis.cadence import detect_step_candidates, estimate_metrics


class StepCounterTests(unittest.TestCase):
    def setUp(self):
        self.t = np.arange(0, 6, 1 / 30)
        self.left = .7 + .04 * np.cos(2 * np.pi * self.t)
        self.right = .7 - .04 * np.cos(2 * np.pi * self.t)

    def test_alternating_feet_do_not_cancel(self):
        events = detect_step_candidates(self.t, self.left, self.right)
        self.assertEqual(len(events), 11)  # Endpoint peaks lack evidence on both sides.
        self.assertTrue(all(a.foot != b.foot for a, b in zip(events, events[1:])))

    def test_stationary_feet(self):
        self.assertEqual(detect_step_candidates(self.t, self.left * 0 + .7, self.left * 0 + .7), [])

    def test_low_visibility(self):
        self.assertEqual(detect_step_candidates(
            self.t, self.left, self.right,
            left_visibility=np.zeros(self.t.size), right_visibility=np.zeros(self.t.size)), [])

    def test_timing_is_independent_of_sample_rate(self):
        dense = detect_step_candidates(self.t, self.left, self.right)
        sparse = detect_step_candidates(self.t[::2], self.left[::2], self.right[::2])
        self.assertEqual(len(dense), len(sparse))
        np.testing.assert_allclose([self.t[e.index] for e in dense],
                                   [self.t[::2][e.index] for e in sparse], atol=1/15)

    def test_no_peak_across_gap(self):
        t = np.array([0, .03, .06, 1, 1.03, 1.06])
        y = np.array([.7, .6, .5, .5, .6, .7])
        self.assertEqual(detect_step_candidates(t, y, y), [])

    def test_metrics_agree_with_candidates(self):
        metrics = estimate_metrics(self.t, self.left, self.right, self.left, self.right)
        self.assertEqual(metrics.detected_steps, len(detect_step_candidates(self.t, self.left, self.right)))
        self.assertIsNone(metrics.speed_m_per_s)

    def test_invalid_inputs(self):
        for t in [self.t[::-1], self.t * np.nan]:
            with self.assertRaises(ValueError):
                detect_step_candidates(t, self.left, self.right)
        with self.assertRaises(ValueError):
            detect_step_candidates(self.t, self.left[:-1], self.right)
        with self.assertRaises(ValueError):
            detect_step_candidates(self.t, self.left, self.right, max_gap_s=-1)
        with self.assertRaises(ValueError):
            estimate_metrics(self.t, self.left, self.right, self.left, self.right,
                             meters_per_normalized_height=-1)


if __name__ == '__main__':
    unittest.main()
