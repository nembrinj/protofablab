"""Default paths for this source-checkout workflow."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
VIDEO_PATH = PROJECT_ROOT / "data" / "walk.mp4"
MODEL_PATH = PROJECT_ROOT / "models" / "pose_landmarker_lite.task"
OUTPUT_DIR = PROJECT_ROOT / "outputs"
