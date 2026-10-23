"""Command-line metrics using the shared pose extractor."""
import argparse
from pathlib import Path

from .cadence import estimate_metrics
from .paths import MODEL_PATH, VIDEO_PATH
from .tracking import extract_pose_tracks


def analyze_video(video_path: Path, model_path: Path, meters_per_normalized_height=None):
    tracks = extract_pose_tracks(video_path, model_path)
    metrics = estimate_metrics(
        tracks.timestamps, tracks.left_x, tracks.right_x, tracks.left_y, tracks.right_y,
        left_visibility=tracks.left_visibility, right_visibility=tracks.right_visibility,
        meters_per_normalized_height=meters_per_normalized_height,
    )
    print(f"Pose coverage: {len(tracks.timestamps)}/{tracks.processed_frames} frames")
    print(f"Detected steps: {metrics.detected_steps}")
    print(f"Cadence: {metrics.cadence_steps_per_minute:.1f} steps/min")
    print(f"Speed: {metrics.speed_m_per_s if metrics.speed_m_per_s is not None else 'not calibrated'}")
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video', nargs='?', type=Path, default=VIDEO_PATH)
    parser.add_argument('--model', type=Path, default=MODEL_PATH)
    parser.add_argument('--meters-per-normalized-height', type=float)
    args = parser.parse_args(argv)
    analyze_video(args.video, args.model, args.meters_per_normalized_height)
