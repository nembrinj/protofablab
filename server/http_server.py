import json
import cv2
import numpy as np
import os

import svgpathtools
import toml
from flask import Flask, request, render_template, redirect, url_for
from svgpathtools import svg2paths, Path

import pipeline.pipeline as pipeline
from silhouette.Graphtec import SilhouetteCameo

UPLOAD_FOLDER = './staticFiles/images'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}

app = Flask(__name__, template_folder='templateFiles', static_folder='staticFiles')
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['SECRET_KEY'] = 'the random string'
config = toml.load('../config.toml')['Server']

dev = SilhouetteCameo(dry_run=False)
state = dev.status()  # hint at loading paper, if not ready.
print(f'status={state}')
print("device version: '%s'" % dev.get_version())

with open('../pipeline/pipeline.json') as f:
    typ = dict[str, str | dict[str, str | list[str] | dict[str, int | float | str]]]
    pipeline_json: typ = json.load(f)


def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


# TODO: Maybe this doesn't work correctly, I couldn't try yet with a frontend
@app.route('/', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        if request.form.getlist('undo'):
            copy_backup()
            return redirect(request.url)

        if request.form.get('send-to-silhouette'):
            send_to_silhouette()
            return redirect(request.url)

        blurs = request.form.getlist('blur') if request.form.getlist('blur') else None
        threshold_type = request.form.get('threshold') if request.form.get('threshold') else None
        invert = bool(request.form.get('invert')) if request.form.get('invert') else None
        dilate_iterations = int(request.form.get('dilate')) if request.form.get('dilate') else None
        erode_iterations = int(request.form.get('erode')) if request.form.get('erode') else None
        contour_type = int(request.form.get('contour')) if request.form.get('contour') else None
        cleanliness = float(request.form.get('clean')) / 10 if request.form.get('clean') else None
        smoothness = float(request.form.get('smooth')) / 10 if request.form.get('smooth') else None

        # check if the post request has the file part
        file = request.files['file']
        if file and allowed_file(file.filename):
            # filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], 'main.png'))
        root(blurs, threshold_type, invert, dilate_iterations, erode_iterations, contour_type, cleanliness, smoothness)
        return redirect(request.url)
    return render_template('index.html')


def root(blurs, threshold_type: str, invert: bool, dilate_iterations: int, erode_iterations: int, contour_type: int,
         cleanliness: float, smoothness: float):
    img = cv2.imread('./staticFiles/images/main.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = gray

    if blurs:
        if 'median' in blurs:
            blur = cv2.medianBlur(blur, 3)
        if 'gaussian' in blurs:
            blur = cv2.GaussianBlur(blur, (5, 5), 0)

    if threshold_type:
        blur = pipeline.threshold(blur, **{'method': threshold_type})['img']

    if invert:
        blur = ~blur

    if dilate_iterations != 0:
        blur = cv2.dilate(blur, np.ones((5, 5), np.uint8), iterations=dilate_iterations)
    if erode_iterations != 0:
        blur = cv2.erode(blur, np.ones((5, 5), np.uint8), iterations=erode_iterations)

    if contour_type:
        contours, _ = cv2.findContours(blur, cv2.RETR_LIST, contour_type)

        if smoothness > 0:
            arr = []
            for cont in contours:
                # TODO: add as argument
                if cv2.arcLength(cont, False) > cleanliness:
                    arr.append(cv2.approxPolyDP(cont, smoothness, False))
            contours = arr

        height, width = gray.shape
        with open('staticFiles/images/main.svg', 'w+') as f:
            svg = pipeline.contours2svg(contours, width, height)
            f.write(
                svg)  # f.write('<svg width="' + str(m) + '" height="' + str(n) + '" xmlns="http://www.w3.org/2000/svg">')  # for c in contours:  #     f.write('<path d="M')  #     for cords in c:  #         x, y = cords[0]  #         f.write(str(x) + ' ' + str(y) + ' ')  #     f.write('"/>')  # f.write('</svg>')

    cv2.imwrite('./staticFiles/images/main.png', blur)


def send_to_silhouette():
    def get_paths(filename):
        paths: list[Path] = svg2paths(filename)[0]
        contours = []

        for path in paths:
            contour = []
            num_points = len(path) + 1

            for i in range(num_points):
                z = path.point(i / (num_points - 1))
                contour.append((z.real, z.imag))

            contours.append(contour)

        return contours

    def scale(contours: list[list[tuple[float, float]]], max_width: float, max_height: float) -> list[list[(int, int)]]:
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        for p in contours:
            for x, y in p:
                min_x = min(min_x, x)
                max_x = max(max_x, x)

                min_y = min(min_y, y)
                max_y = max(max_y, y)

        w = max_x - min_x
        h = max_y - min_y

        factor_width = max_width / w
        factor_height = max_height / h

        if factor_width > factor_height:
            factor_height = factor_width
        else:
            factor_width = factor_height

        scaled = []
        for p in contours:
            path = []
            for x, y in p:
                x = (x - min_x) * factor_width
                y = (y - min_y) * factor_height

                path.append((x, y))

            scaled.append(path)

        return scaled

    with open('staticFiles/images/main.svg') as filename:
        contours = get_paths(filename)
        contours = scale(contours, max_width=100, max_height=100)

        dev.setup(media=132, pen=True, pressure=10, speed=3)
        dev.plot(pathlist=contours, offset=(5, 5))


def copy_backup():
    img = cv2.imread('./staticFiles/images/backup.JPG')
    cv2.imwrite('./staticFiles/images/main.png', img)
    if os.path.exists('./staticFiles/images/main.svg'):
        os.remove('./staticFiles/images/main.svg')


# TODO: Add actual routes when needed


if __name__ == '__main__':
    copy_backup()
    app.run(host=config['host'], port=config['port'])
