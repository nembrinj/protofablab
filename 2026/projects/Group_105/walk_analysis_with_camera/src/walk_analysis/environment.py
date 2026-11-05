"""Check that the MediaPipe development environment is ready."""

import platform
from mediapipe.tasks.python.core.mediapipe_c_bindings import load_raw_library

import mediapipe as mp
import cv2
import numpy as np


def main(argv=None) -> None:
    import argparse
    argparse.ArgumentParser(description=__doc__).parse_args(argv)
    print(f"Python {platform.python_version()} ({platform.machine()})")
    print(f"MediaPipe {mp.__version__}")
    print(f"OpenCV {cv2.__version__}")
    print(f"NumPy {np.__version__}")
    load_raw_library()  # Check native dependencies as well as Python imports.
    print("MediaPipe Tasks native library ready")
