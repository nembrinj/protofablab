import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from io import BytesIO
from zipfile import ZipFile
import numpy as np
from walk_analysis.pose_models import coco_points, model_status, MODEL_SPECS
from walk_analysis.download_model import _download


class PoseModelsTests(unittest.TestCase):
    def test_coco_mapping_preserves_leg_sides_and_omits_feet(self):
        xy = np.arange(34).reshape(17, 2)
        pts = coco_points(xy, np.ones(17), 100, 200)
        self.assertEqual((pts[27].x, pts[27].y), (0.30, 0.155))
        self.assertEqual((pts[28].x, pts[28].y), (0.32, 0.165))
        self.assertTrue(all(pts[i].visibility == 0 for i in (29, 30, 31, 32)))

    def test_status_requires_rtmpose_person_detector(self):
        with TemporaryDirectory() as d:
            (Path(d) / MODEL_SPECS['rtmpose-m']['file']).touch()
            self.assertFalse(model_status(Path(d))['rtmpose-m']['available'])

    def test_invalid_download_preserves_existing_model(self):
        with TemporaryDirectory() as d:
            target = Path(d) / 'model.tflite'
            target.write_bytes(b'previous model')
            with patch('walk_analysis.download_model.urlopen', return_value=BytesIO(b'<html>error</html>')):
                with self.assertRaises(ValueError):
                    _download(dict(url='https://example.test/model'), target)
            self.assertEqual(target.read_bytes(), b'previous model')

    def test_zip_reads_model_without_extracting_paths(self):
        archive = BytesIO()
        with ZipFile(archive, 'w') as z:
            z.writestr('../../outside.onnx', b'model payload')
        with TemporaryDirectory() as d:
            target = Path(d) / 'model.onnx'
            with patch('walk_analysis.download_model.urlopen', return_value=BytesIO(archive.getvalue())):
                _download(dict(url='https://example.test/model.zip'), target)
            self.assertEqual(target.read_bytes(), b'model payload')


if __name__ == '__main__':
    unittest.main()
