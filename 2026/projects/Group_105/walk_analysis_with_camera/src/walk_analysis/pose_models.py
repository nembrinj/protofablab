"""Optional pose backends for the live comparison notebook.

COCO-17 points map into MediaPipe's 33 slots for the existing overlay. Missing
heels/toes are explicitly zero-score placeholders, never inferred from ankles.
"""
from dataclasses import dataclass
from importlib.util import find_spec
from pathlib import Path
import cv2
import numpy as np

from .paths import MODEL_PATH

MP_NAMES = ("lite", "full", "heavy")
MODEL_SPECS = {
    name: dict(label=f"MediaPipe {name.title()}", backend="mediapipe",
               file=f"pose_landmarker_{name}.task", module="mediapipe",
               url=("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
                    f"pose_landmarker_{name}/float16/1/pose_landmarker_{name}.task"))
    for name in MP_NAMES
}
MODEL_SPECS.update({
    "yolov8n": dict(label="YOLOv8n Pose", backend="yolo", module="ultralytics", file="yolov8n-pose.pt",
                    url="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-pose.pt"),
    "yolo11n": dict(label="YOLO11n Pose", backend="yolo", module="ultralytics", file="yolo11n-pose.pt",
                   url="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt"),
    "movenet-thunder": dict(label="MoveNet Thunder", backend="movenet", module="ai_edge_litert", file="movenet_thunder.tflite",
        url="https://tfhub.dev/google/lite-model/movenet/singlepose/thunder/tflite/float16/4?lite-format=tflite"),
    "rtmpose-m": dict(label="RTMPose-m + YOLOX-m", backend="rtmpose", module="rtmlib", file="rtmpose_m.onnx",
        url="https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-m_simcc-body7_pt-body7_420e-256x192-e48f03d0_20230504.zip"),
    "rtmo-m": dict(label="RTMO-m", backend="rtmo", module="rtmlib", file="rtmo_m.onnx",
        url="https://download.openmmlab.com/mmpose/v1/projects/rtmo/onnx_sdk/rtmo-m_16xb16-600e_body7-640x640-39e78cc4_20231211.zip"),
})
YOLOX_SPEC = dict(file="yolox_m.onnx", url="https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/yolox_m_8xb8-300e_humanart-c2c7a14a.zip")
COCO_TO_MP = (0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    visibility: float = 0.0  # Native score; meaning depends on backend.


def coco_points(xy, scores, width, height):
    """Map one person's pixel COCO-17 points to normalized overlay coordinates."""
    points = [Point() for _ in range(33)]
    for i, slot in enumerate(COCO_TO_MP):
        if np.isfinite(xy[i]).all() and np.isfinite(scores[i]):
            points[slot] = Point(float(xy[i, 0] / width), float(xy[i, 1] / height), float(scores[i]))
    return points


def model_status(model_dir=MODEL_PATH.parent):
    statuses = {}
    for name, spec in MODEL_SPECS.items():
        missing = not (Path(model_dir) / spec['file']).is_file()
        if name == 'rtmpose-m':
            missing |= not (Path(model_dir) / YOLOX_SPEC['file']).is_file()
        reason = 'download weights' if missing else ''
        if not reason and find_spec(spec['module']) is None:
            reason = 'install optional backend'
        if not reason and spec['backend'] in ('rtmpose', 'rtmo') and find_spec('onnxruntime') is None:
            reason = 'install onnxruntime'
        statuses[name] = dict(label=spec['label'], available=not reason, reason=reason)
    return statuses


class PoseDetector:
    """One CPU model at a time. No identity tracking across multiple people."""
    def __init__(self, name, model_dir=MODEL_PATH.parent):
        spec = MODEL_SPECS[name]
        self.backend = spec['backend']
        path = Path(model_dir) / spec['file']
        if not path.is_file():
            raise FileNotFoundError(f'Missing {path}; run download-model --model {name}')
        self.score_kind = 'visibility' if self.backend == 'mediapipe' else 'keypoint_confidence'
        if self.backend == 'mediapipe':
            import mediapipe as mp
            self.model = mp.tasks.vision.PoseLandmarker.create_from_options(
                mp.tasks.vision.PoseLandmarkerOptions(
                    base_options=mp.tasks.BaseOptions(model_asset_path=str(path)),
                    running_mode=mp.tasks.vision.RunningMode.VIDEO, num_poses=1,
                    min_pose_detection_confidence=0.5, min_tracking_confidence=0.5))
        elif self.backend == 'yolo':
            from ultralytics import YOLO
            self.model = YOLO(str(path), task='pose')
        elif self.backend == 'movenet':
            from ai_edge_litert.interpreter import Interpreter
            self.model = Interpreter(model_path=str(path), num_threads=2)
            self.model.allocate_tensors()
        elif self.backend == 'rtmpose':
            from rtmlib import Body
            self.model = Body(det=str(Path(model_dir) / YOLOX_SPEC['file']), det_input_size=(640, 640),
                              pose=str(path), pose_input_size=(192, 256),
                              to_openpose=False, backend='onnxruntime', device='cpu')
        else:
            from rtmlib import RTMO
            self.model = RTMO(str(path), model_input_size=(640, 640),
                              to_openpose=False, backend='onnxruntime', device='cpu')

    def detect(self, frame, timestamp):
        h, w = frame.shape[:2]
        if self.backend == 'mediapipe':
            import mediapipe as mp
            result = self.model.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,
                data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)), timestamp)
            return result.pose_landmarks[0] if result.pose_landmarks else []
        if self.backend == 'yolo':
            result = self.model.predict(frame, device='cpu', conf=0.25, imgsz=640, verbose=False)[0]
            if result.keypoints is None or len(result.keypoints) == 0:
                return []
            index = int(result.boxes.conf.argmax())
            xy = result.keypoints.xy[index].cpu().numpy()
            scores = result.keypoints.conf[index].cpu().numpy()
        elif self.backend == 'movenet':
            # Centered square letterbox; invert the padding before returning points.
            inp = self.model.get_input_details()[0]
            size = int(inp['shape'][1])
            scale = min(size / w, size / h)
            rw, rh = max(1, round(w * scale)), max(1, round(h * scale))
            left, top = (size-rw)//2, (size-rh)//2
            image = np.zeros((size, size, 3), dtype=np.uint8)
            image[top:top+rh, left:left+rw] = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), (rw, rh))
            self.model.set_tensor(inp['index'], image[None].astype(inp['dtype']))
            self.model.invoke()
            out = self.model.get_tensor(self.model.get_output_details()[0]['index'])[0, 0]
            xy = np.column_stack(((out[:, 1]*size-left)/(rw/w), (out[:, 0]*size-top)/(rh/h)))
            scores = out[:, 2]
        else:
            if self.backend == 'rtmpose':
                # RTMLib's Body falls back to full-image pose if its detector is empty.
                # Explicitly return NO POSE instead of creating a phantom person.
                boxes = self.model.det_model(frame)
                if len(boxes) == 0:
                    return []
                keypoints, confidences = self.model.pose_model(frame, bboxes=boxes)
            else:
                keypoints, confidences = self.model(frame)
            if len(keypoints) == 0:
                return []
            index = int(np.argmax(np.mean(confidences, axis=1)))
            xy, scores = keypoints[index], confidences[index]
        # Single-pose networks always emit points, even for an empty image.
        # This is an application heuristic, not a native person-detection score.
        if np.count_nonzero(np.asarray(scores) >= 0.2) < 4:
            return []
        return coco_points(xy, scores, w, h)

    def close(self):
        if self.backend == 'mediapipe':
            self.model.close()
        self.model = None
