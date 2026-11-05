"""Signal processing helpers for a first walking-cadence experiment."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GaitMetrics:
    """Estimated metrics for the analyzed time window."""

    cadence_steps_per_minute: float
    speed_m_per_s: float | None
    detected_steps: int


@dataclass(frozen=True)
class StepCandidate:
    index: int
    foot: str
    prominence: float


def detect_step_candidates(
    timestamps_s: np.ndarray, left_y: np.ndarray, right_y: np.ndarray,
    *, left_visibility=None, right_visibility=None,
    min_visibility: float = 0.5, smoothing_s: float = 0.10,
    max_gap_s: float = 0.20, min_same_foot_s: float = 0.40,
    min_step_s: float = 0.18, min_prominence: float = 0.003,
) -> list[StepCandidate]:
    """Find separate foot-lift candidates; never smooth across tracking gaps.

    Times are seconds, heights are normalized image coordinates. Candidates are
    foot lifts, not confirmed ground contacts. Visibility is optional for callers
    that only have coordinate tracks; nonfinite coordinates always break tracks.
    """
    parameters = [smoothing_s, max_gap_s, min_same_foot_s, min_step_s, min_prominence]
    if not all(np.isfinite(value) and value > 0 for value in parameters):
        raise ValueError("time windows and prominence must be finite and positive")
    if not np.isfinite(min_visibility) or not 0 <= min_visibility <= 1:
        raise ValueError("min_visibility must be between zero and one")
    times = np.asarray(timestamps_s, dtype=float)
    if times.ndim != 1 or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("timestamps must be finite and strictly increasing")
    candidates = []
    for foot, values, visibility in [('L', left_y, left_visibility), ('R', right_y, right_visibility)]:
        values = np.asarray(values, dtype=float)
        if values.shape != times.shape:
            raise ValueError("timestamps and ankle tracks must have equal lengths")
        valid = np.isfinite(values)
        if visibility is not None:
            visibility = np.asarray(visibility, dtype=float)
            if visibility.shape != times.shape:
                raise ValueError("visibility and ankle tracks must have equal lengths")
            valid &= np.isfinite(visibility) & (visibility >= min_visibility)
        indices = np.flatnonzero(valid)
        breaks = np.flatnonzero((np.diff(indices) > 1) | (np.diff(times[indices]) > max_gap_s)) + 1
        for segment in np.split(indices, breaks):
            if segment.size < 3:
                continue
            t = times[segment]
            raw = -values[segment]
            smooth = np.array([np.mean(raw[np.abs(t - time) <= smoothing_s / 2]) for time in t])
            threshold = max(min_prominence, float(np.percentile(smooth, 75) - np.percentile(smooth, 25)) * 0.2)
            peaks = np.flatnonzero((smooth[1:-1] > smooth[:-2]) & (smooth[1:-1] >= smooth[2:])) + 1
            local = []
            for peak in peaks:
                before = smooth[(t >= t[peak] - 0.35) & (t < t[peak])]
                after = smooth[(t <= t[peak] + 0.35) & (t > t[peak])]
                if not before.size or not after.size:
                    continue
                prominence = float(smooth[peak] - max(before.min(), after.min()))
                if prominence >= threshold:
                    local.append(StepCandidate(int(segment[peak]), foot, prominence))
            selected = []
            for candidate in sorted(local, key=lambda event: event.prominence, reverse=True):
                if all(abs(times[candidate.index] - times[event.index]) >= min_same_foot_s for event in selected):
                    selected.append(candidate)
            candidates.extend(selected)
    # Reject near-simultaneous peaks from both feet, keeping the stronger one.
    selected = []
    for candidate in sorted(candidates, key=lambda event: event.prominence, reverse=True):
        if all(abs(times[candidate.index] - times[event.index]) >= min_step_s for event in selected):
            selected.append(candidate)
    return sorted(selected, key=lambda event: event.index)


def step_peak_indices(timestamps_s: np.ndarray, left_y: np.ndarray, right_y: np.ndarray, **kwargs) -> np.ndarray:
    """Return indices of the same events used by metrics and debug overlays."""
    return np.asarray([event.index for event in detect_step_candidates(timestamps_s, left_y, right_y, **kwargs)], dtype=int)


def estimate_metrics(
    timestamps_s: np.ndarray,
    left_ankle_x: np.ndarray,
    right_ankle_x: np.ndarray,
    left_ankle_y: np.ndarray,
    right_ankle_y: np.ndarray,
    *,
    meters_per_normalized_height: float | None = None,
    left_visibility=None,
    right_visibility=None,
) -> GaitMetrics:
    """Estimate cadence and optional forward speed from normalized ankle tracks.

    The ankle y-coordinate is inverted so a foot lift becomes a peak. Speed is
    only returned when the caller supplies a scene scale in meters per normalized
    image-height unit and is based on the midpoint displacement of both ankles.
    """
    timestamps_s, left_ankle_x, right_ankle_x, left_ankle_y, right_ankle_y = [
        np.asarray(values, dtype=float) for values in
        (timestamps_s, left_ankle_x, right_ankle_x, left_ankle_y, right_ankle_y)
    ]
    if any(values.ndim != 1 for values in
           (timestamps_s, left_ankle_x, right_ankle_x, left_ankle_y, right_ankle_y)):
        raise ValueError("timestamps and ankle tracks must be one-dimensional")
    if meters_per_normalized_height is not None and (
        not np.isfinite(meters_per_normalized_height) or meters_per_normalized_height <= 0
    ):
        raise ValueError("scene scale must be finite and positive")
    if not (
        timestamps_s.size
        == left_ankle_x.size
        == right_ankle_x.size
        == left_ankle_y.size
        == right_ankle_y.size
    ):
        raise ValueError("timestamps and ankle tracks must have equal lengths")
    if timestamps_s.size < 3:
        raise ValueError("at least three frames are required")
    duration_s = float(timestamps_s[-1] - timestamps_s[0])
    if duration_s <= 0:
        raise ValueError("timestamps must increase")

    peaks = step_peak_indices(timestamps_s, left_ankle_y, right_ankle_y,
                              left_visibility=left_visibility, right_visibility=right_visibility)
    cadence = float(peaks.size / duration_s * 60.0)

    speed = None
    if meters_per_normalized_height is not None:
        midpoint = np.nanmean(np.vstack((left_ankle_x, right_ankle_x)), axis=0)
        displacement = abs(float(midpoint[-1] - midpoint[0]))
        speed = displacement * meters_per_normalized_height / duration_s
    return GaitMetrics(cadence, speed, int(peaks.size))
