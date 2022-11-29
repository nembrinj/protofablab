import base64

import cv2
import numpy as np


def to_json_value(img, format: str = '.png') -> str:
    _, buf = cv2.imencode(format, img)

    return base64.b64encode(buf).decode()


def from_json_value(val):
    jpg_original = base64.b64decode(val)
    jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)

    return cv2.imdecode(jpg_as_np, flags=1)
