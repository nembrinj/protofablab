import base64
import json

import cv2
import numpy as np

with open('../pipeline/pipeline.json') as f:
    typ = dict[str, str | dict[str, str | list[str] | dict[str, int | float | str]]]
    pipeline_json: typ = json.load(f)


def to_json_value(img, form: str = '.png') -> str:
    """
    Converts an image into a base64-encoded string for JSON.
    :param img: the image to convert
    :param form: the file ending format (default = '.png')
    :return: base64-encoded image as string
    """
    _, buf = cv2.imencode(form, img)

    return base64.b64encode(buf).decode()


def from_json_value(val: str):
    """
    Converts a base64-encoded string into an image object.
    :param val: the base64-encoded image string
    :return: image in RGB format
    """
    img = base64.b64decode(val)
    image_np = np.frombuffer(img, dtype=np.uint8)

    return cv2.cvtColor(cv2.imdecode(image_np, flags=cv2.IMREAD_COLOR), cv2.BGR2RGB)


def contour_len(contours: list) -> int:
    """
    Returns the actual length / number of segments in contours object.
    :param contours: the contours
    :return: number of segments
    """
    return sum(map(len, contours))


def grayscale(img, **kwargs) -> dict:
    """
    Converts an RGB image into grayscale
    :param img: the image to convert
    :param kwargs: ignored
    :return: grayscale image in a dict
    """
    return {'img': cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)}


def rgb(img, **kwargs) -> dict:
    """
    Converts a grayscale image into RGB.
    :param img:  the image to convert
    :param kwargs: ignored
    :return: RGB image in a dict
    """
    return {'img': cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)}


def blur(img, **kwargs) -> dict:
    """
    Blurs an image according to given parameters.
    :param img: the image to blur
    :param kwargs: parameters for the performed blur.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: blurred image in a dict
    """
    match kwargs['method']:
        case 'gaussian':
            ksize: int = kwargs.setdefault('kernel', pipeline_json['blur']['gaussian']['kernel']['recommended'])
            sigma: int = kwargs.setdefault('sigma', pipeline_json['blur']['gaussian']['sigma']['recommended'])
            return {'img': cv2.GaussianBlur(img, (ksize, ksize), sigma)}
        case 'median':
            ksize: int = kwargs.setdefault('kernel', pipeline_json['blur']['median']['kernel']['recommended'])
            return {'img': cv2.medianBlur(img, ksize)}


def threshold(img, **kwargs) -> dict:
    """
    Applies thresholding to an image according to given parameters.
    :param img: the image to threshold
    :param kwargs: parameters for the performed threshold.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: thresholded image in a dict
    """
    match kwargs['method']:
        case 'manual':
            t: int = kwargs.setdefault('kernel', pipeline_json['threshold']['manual']['threshold']['recommended'])
            return {'img': cv2.threshold(img, t, 255, cv2.THRESH_BINARY)[1]}
        case 'otsu':
            return {'img': cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]}
        case 'mean':
            block_size: int = kwargs.setdefault('block_size',
                                                pipeline_json['threshold']['mean']['block_size']['recommended'])
            c: int = kwargs.setdefault('c', pipeline_json['threshold']['mean']['c']['recommended'])
            return {
                'img': cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block_size, c)}
        case 'gaussian':
            block_size: int = kwargs.setdefault('block_size',
                                                pipeline_json['threshold']['mean']['block_size']['recommended'])
            c: int = kwargs.setdefault('c', pipeline_json['threshold']['mean']['c']['recommended'])
            return {
                'img': cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block_size,
                                             c)}


def edge_detect(img, **kwargs) -> dict:
    """
    Performs edge detection on an image according to given parameters.
    Note that applying `find_contours` will then find contours for both sides of the edge.
    If this effect is not wanted, try applying `find_contours` directly.
    :param img: the image
    :param kwargs: parameters for the edge detection.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: edge-ified image in a dict
    """
    min: int = kwargs.setdefault('min', pipeline_json['edge_detect']['min']['recommended'])
    max: int = kwargs.setdefault('max', pipeline_json['edge_detect']['max']['recommended'])
    return {'img': cv2.Canny(img, min, max)}


def dilate(img, **kwargs) -> dict:
    """
    Performs dilation on an image according to given parameters.
    :param img: the image to dilate
    :param kwargs: parameters for dilation.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: dilated image in a dict
    """
    ksize: int = kwargs.setdefault('kernel', pipeline_json['dilate']['kernel']['recommended'])
    kernel = np.ones((ksize, ksize), np.uint8)
    iterations: int = kwargs.setdefault('iterations', pipeline_json['dilate']['iterations']['recommended'])

    return {'img': cv2.dilate(img, kernel, iterations)}


def erode(img, **kwargs) -> dict:
    """
    Performs erosion on an image according to given parameters.
    :param img: the image to erode
    :param kwargs: parameters for erosion.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: eroded image in a dict
    """
    ksize: int = kwargs.setdefault('kernel', pipeline_json['erode']['kernel']['recommended'])
    kernel = np.ones((ksize, ksize), np.uint8)
    iterations: int = kwargs.setdefault('iterations', pipeline_json['erode']['iterations']['recommended'])

    return {'img': cv2.erode(img, kernel, iterations)}


def find_contours(img, **kwargs) -> dict:
    """
    Finds contours in an image according to the algorithm in given parameters.
    Note that this method finds two contours for an edge-detected image.
    If this effect is not wanted, skip over any prior edge-detection.
    :param img: the image to find contours in
    :param kwargs: parameters for contour finding.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: contours and its length in a dict
    """
    m = kwargs.setdefault('method', pipeline_json['find_contours']['recommended'])
    try:
        method = int(m)
    except:
        method = ['none', 'simple', 'tc89_l1', 'tc89_kcos'].index(m)
    contours = cv2.findContours(img, cv2.RETR_LIST, method)[0]
    return {'contours': contours, 'length': contour_len(contours)}


def smooth_contours(img=None, **kwargs) -> dict:
    """
    Smooths contours according to given parameters.
    :param img: ignored
    :param kwargs: parameters for contour smoothing.
                   Read the documentation in the `pipeline.json` and `README.md` of this module.
    :return: smoothed contours and its length in a dict
    """
    contours = list(kwargs['contours'])
    accuracy: float = kwargs.setdefault('accuracy', pipeline_json['smooth_contours']['accuracy']['recommended'])

    for i in range(0, len(contours)):
        contours[i] = cv2.approxPolyDP(contours[i], accuracy, False)

    return {'contours': contours, 'length': contour_len(contours)}


def dimensions(contours: list[list]) -> (float, float):
    """
    Returns the dimensions (width, height) of given contours.
    :param contours: the contours
    :return: width and height of all contours
    """
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


def contours2svg(contours: list[list]) -> str:
    """
    Converts contours to SVG specifications.
    :param contours: the contours
    :return: SVG specification as a str
    """
    width, height = dimensions(contours)
    s = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">\n'
    for path in contours:
        if len(path) >= 2:
            s += '    <path d="M'
            for idx, point in enumerate(path):
                x, y = point[0, 0], point[0, 1]
                if idx == 0:
                    s += f'{x} {y}'
                else:
                    s += f' {x} {y}'

            s += '" style="fill:none; stroke:black; stroke-width:5"/>\n'

    return s + '</svg>'


def call_pipeline(args: dict) -> dict:
    """
    Calls the pipeline according to the content and parameters.
    :param args: the arguments of how a pipeline call.
                 Must contain at least the key 'img' and 'pipeline' with additional call details.
    :return: result of pipeline call
    """
    # TODO: should the image be an actual image object or a base64-encoded str?

    img = args['img']
    pipeline = args['pipeline']

    assert len(pipeline) == 1

    for method, kwargs in pipeline:
        fun = locals()[method]

        result = fun(img=img, kwargs=kwargs)
        if 'img' in result:
            result['img'] = to_json_value(result['img'])

        return result
