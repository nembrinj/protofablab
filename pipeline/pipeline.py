import json
import base64

import cv2
import numpy as np

with open('../pipeline/pipeline.json') as f:
    typ = dict[str, str | dict[str, str | list[str] | dict[str, int | float | str]]]
    pipeline_json: typ = json.load(f)



def to_json_value(img, form: str = '.png') -> str:
    _, buf = cv2.imencode(form, img)

    return base64.b64encode(buf).decode()


def from_json_value(val):
    jpg_original = base64.b64decode(val)
    jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)

    return cv2.imdecode(jpg_as_np, flags=1)


def contour_len(contours: list) -> int:
    return sum(map(len, contours))


def grayscale(img, **kwargs) -> dict:
    return {'img': cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)}


def rgb(img, **kwargs) -> dict:
    return {'img': cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)}


def blur(img, **kwargs) -> dict:
    match kwargs['method']:
        case 'gaussian':
            ksize: int = kwargs.setdefault('kernel', pipeline_json['blur']['gaussian']['kernel']['recommended'])
            sigma: int = kwargs.setdefault('sigma', pipeline_json['blur']['gaussian']['sigma']['recommended'])
            return {'img': cv2.GaussianBlur(img, (ksize, ksize), sigma)}
        case 'median':
            ksize: int = kwargs.setdefault('kernel', pipeline_json['blur']['median']['kernel']['recommended'])
            return {'img': cv2.medianBlur(img, ksize)}


def threshold(img, **kwargs) -> dict:
    match kwargs['method']:
        case 'manual':
            t: int = kwargs.setdefault('kernel', pipeline_json['threshold']['manual']['threshold']['recommended'])
            return {'img': cv2.threshold(img, t, 255, cv2.THRESH_BINARY)[1]}
        case 'otsu':
            return {'img': cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]}
        case 'mean':
            block_size: int = kwargs.setdefault('block_size', pipeline_json['threshold']['mean']['block_size']['recommended'])
            c: int = kwargs.setdefault('c', pipeline_json['threshold']['mean']['c']['recommended'])
            return {
                'img': cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block_size, c)}
        case 'gaussian':
            block_size: int = kwargs.setdefault('block_size', pipeline_json['threshold']['mean']['block_size']['recommended'])
            c: int = kwargs.setdefault('c', pipeline_json['threshold']['mean']['c']['recommended'])
            return {
                'img': cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block_size,
                                             c)}


def edge_detect(img, **kwargs) -> dict:
    min: int = kwargs.setdefault('min', pipeline_json['edge_detect']['min']['recommended'])
    max: int = kwargs.setdefault('max', pipeline_json['edge_detect']['max']['recommended'])
    return {'img': cv2.Canny(img, min, max)}


def dilate(img, **kwargs) -> dict:
    ksize: int = kwargs.setdefault('kernel', pipeline_json['dilate']['kernel']['recommended'])
    kernel = np.ones((ksize, ksize), np.uint8)
    iterations: int = kwargs.setdefault('iterations', pipeline_json['dilate']['iterations']['recommended'])

    return {'img': cv2.dilate(img, kernel, iterations)}


def erode(img, **kwargs) -> dict:
    ksize: int = kwargs.setdefault('kernel', pipeline_json['erode']['kernel']['recommended'])
    kernel = np.ones((ksize, ksize), np.uint8)
    iterations: int = kwargs.setdefault('iterations', pipeline_json['erode']['iterations']['recommended'])

    return {'img': cv2.erode(img, kernel, iterations)}


def find_contours(img, **kwargs) -> dict:
    m = kwargs.setdefault('method', pipeline_json['find_contours']['recommended'])
    try:
        method = int(m)
    except:
        method = ['none', 'simple', 'tc89_l1', 'tc89_kcos'].index(m)
    contours = cv2.findContours(img, cv2.RETR_LIST, method)[0]
    return {'contours': contours, 'length': contour_len(contours)}


def smooth_contours(img=None, **kwargs) -> dict:
    contours = list(kwargs['contours'])
    accuracy: float = kwargs.setdefault('accuracy', pipeline_json['smooth_contours']['accuracy']['recommended'])

    for i in range(0, len(contours)):
        contours[i] = cv2.approxPolyDP(contours[i], accuracy, True)

    return {'contours': contours, 'length': contour_len(contours)}


def dimensions(contours) -> (float, float):
    min_x = float('inf')
    max_x = float('-inf')
    min_y = float('inf')
    max_y = float('-inf')
    for path in contours:
        for point in path:
            x, y = point[0, 0], point[0, 1]
            min_x = min(min_x, x)
            max_x = max(max_x, x)

            min_y = min(min_y, y)
            max_y = max(max_y, y)

    return max_x - min_x, max_y - min_y


def contours2svg(contours) -> str:
    width, height = dimensions(contours)
    s = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">'
    for path in contours:
        if len(path) > 2:
            s += '<path d="M'
            for point in path:
                x, y = point[0, 0], point[0, 1]
                s += f'{x} {y} '

            s += '" style="stroke:black"/>'

    return s + '</svg>'


def call_pipeline(args) -> dict:
    img = args['img']
    pipeline = args['pipeline']

    assert len(pipeline) == 1

    for method, kwargs in pipeline:
        fun = locals()[method]

        result = fun(img=img, kwargs=kwargs)
        if 'img' in result:
            result['img'] = to_json_value(result['img'])

        return result
