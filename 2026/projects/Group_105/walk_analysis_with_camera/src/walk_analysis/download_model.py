"""Download the small official Pose Landmarker model."""

import argparse
from pathlib import Path
from urllib.request import urlopen

from .paths import MODEL_PATH

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
)


def download_model(destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with urlopen(MODEL_URL, timeout=60) as response:
        content = response.read()
    temporary = destination.with_suffix(".task.tmp")
    temporary.write_bytes(content)
    temporary.replace(destination)
    print(f"Saved {destination}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=MODEL_PATH)
    args = parser.parse_args(argv)
    download_model(args.output)
