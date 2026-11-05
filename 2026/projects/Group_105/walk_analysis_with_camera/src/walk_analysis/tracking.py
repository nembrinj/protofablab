"""Shared video metadata and MediaPipe pose extraction for every entry point."""
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import mediapipe as mp
import numpy as np


@dataclass(frozen=True)
class VideoInfo:
    fps: float
    frame_count: int
    width: int
    height: int


def read_video_info(path: Path) -> VideoInfo:
    capture = cv2.VideoCapture(str(path))
    try:
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open video: {path}")
        info = VideoInfo(capture.get(cv2.CAP_PROP_FPS),
                         int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
                         int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                         int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if not np.isfinite(info.fps) or info.fps <= 0 or min(info.width, info.height) <= 0:
            raise RuntimeError(f"Video has invalid frame rate or dimensions: {path}")
        return info
    finally:
        capture.release()


@dataclass
class PoseTracks:
    info: VideoInfo
    processed_frames: int
    frame_indices: np.ndarray
    timestamps: np.ndarray
    left_x: np.ndarray
    right_x: np.ndarray
    left_y: np.ndarray
    right_y: np.ndarray
    left_visibility: np.ndarray
    right_visibility: np.ndarray
    poses: list[Any] | None


def extract_pose_tracks(video_path: Path, model_path: Path, *, keep_poses: bool = False) -> PoseTracks:
    """Extract one pose per frame. Only the overlay exporter retains full poses."""
    if not model_path.is_file():
        raise FileNotFoundError(f"Missing model: {model_path}. Run python -m walk_analysis download-model")
    info = read_video_info(video_path)
    capture = cv2.VideoCapture(str(video_path))
    rows = []
    poses = [] if keep_poses else None
    processed = 0
    options = mp.tasks.vision.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
        running_mode=mp.tasks.vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.5, min_tracking_confidence=0.5,
    )
    try:
        with mp.tasks.vision.PoseLandmarker.create_from_options(options) as detector:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                result = detector.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)),
                    round(processed / info.fps * 1000),
                )
                pose = result.pose_landmarks[0] if result.pose_landmarks else None
                if poses is not None:
                    poses.append(pose)
                if pose:
                    left, right = pose[27], pose[28]
                    rows.append([processed, left.x, right.x, left.y, right.y,
                                 left.visibility, right.visibility])
                processed += 1
    finally:
        capture.release()
    if len(rows) < 3:
        raise RuntimeError("Fewer than three pose frames; check framing and lighting")
    data = np.asarray(rows, dtype=float)
    indices = data[:, 0].astype(int)
    return PoseTracks(info, processed, indices, indices / info.fps,
                      *[data[:, col] for col in range(1, 7)], poses)
