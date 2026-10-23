"""Checks for shared video validation and source-checkout defaults."""
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from walk_analysis.paths import MODEL_PATH, OUTPUT_DIR, PROJECT_ROOT, VIDEO_PATH
from walk_analysis.tracking import extract_pose_tracks, read_video_info


class TrackingTests(unittest.TestCase):
    def test_defaults_resolve_to_checkout(self):
        self.assertTrue((PROJECT_ROOT / 'pyproject.toml').is_file())
        self.assertEqual(VIDEO_PATH, PROJECT_ROOT / 'data/walk.mp4')
        self.assertEqual(MODEL_PATH, PROJECT_ROOT / 'models/pose_landmarker_lite.task')
        self.assertEqual(OUTPUT_DIR, PROJECT_ROOT / 'outputs')

    @patch('walk_analysis.tracking.cv2.VideoCapture')
    def test_unreadable_video_releases_capture(self, factory):
        capture = Mock()
        capture.isOpened.return_value = False
        factory.return_value = capture
        with self.assertRaisesRegex(RuntimeError, 'Cannot open video'):
            read_video_info(Path('missing.mp4'))
        capture.release.assert_called_once()

    @patch('walk_analysis.tracking.cv2.VideoCapture')
    def test_invalid_metadata_releases_capture(self, factory):
        capture = Mock()
        capture.isOpened.return_value = True
        capture.get.side_effect = [float('nan'), 10, 640, 480]
        factory.return_value = capture
        with self.assertRaisesRegex(RuntimeError, 'invalid frame rate'):
            read_video_info(Path('invalid.mp4'))
        capture.release.assert_called_once()

    def test_missing_model_has_actionable_error(self):
        with self.assertRaisesRegex(FileNotFoundError, 'download-model'):
            extract_pose_tracks(Path('unused.mp4'), Path('/tmp/nonexistent-walk-model.task'))
