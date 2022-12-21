import json
import cv2
import numpy as np
import os
import toml
from flask import Flask, request, render_template, redirect
from svgpathtools import svg2paths, Path

import pipeline.pipeline as pipeline

# from silhouette.Graphtec import SilhouetteCameo

UPLOAD_FOLDER = './staticFiles/images/pngs/'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}
IMG_AMT = 4

app = Flask(__name__, template_folder='templateFiles', static_folder='staticFiles')
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['SECRET_KEY'] = 'ProtoFabLab'
config = toml.load('../config.toml')['Server']

# dev = SilhouetteCameo(dry_run=False)
# state = dev.status()  # hint at loading paper, if not ready.

with open('../pipeline/pipeline.json') as f:
    typ = dict[str, str | dict[str, str | list[str] | dict[str, int | float | str]]]
    pipeline_json: typ = json.load(f)


def allowed_file(filename):
    return '.' in filename and filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


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

        # dev.setup(media=132, pen=True, pressure=10, speed=3)
        # dev.plot(pathlist=contours, offset=(5, 5))


@app.route('/', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        if request.form.getlist('undo'):
            undo()
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
        cleanliness = float(request.form.get('clean')) if request.form.get('clean') else None
        smoothness = float(request.form.get('smooth')) / 10 if request.form.get('smooth') else None

        # check if the post request has the file part
        file = request.files['file']
        if file and allowed_file(file.filename):
            # filename = secure_filename(file.filename)
            delete_files_in_folder('./staticFiles/images/pngs/')
            delete_files_in_folder('./staticFiles/images/svgs/')
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], '0.png'))
            height, width = cv2.imread(UPLOAD_FOLDER + '0.png').shape[:2]
            svg = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg"></svg>'
            write_svg(svg, 0)
            return redirect(request.url)
        root(blurs, threshold_type, invert, dilate_iterations, erode_iterations, contour_type, cleanliness, smoothness)
        return redirect(request.url)
    return render_template('index.html')


def root(blurs, threshold_type: str, invert: bool, dilate_iterations: int, erode_iterations: int, contour_type: int,
         cleanliness: float, smoothness: float):
    idx = get_latest_img()
    img = cv2.imread('./staticFiles/images/pngs/' + str(idx) + '.png')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    svg = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg"></svg>'
    blur = gray
    print(blurs)
    if blurs is not ['1', '1']:
        if int(blurs[0]) > 1:
            blur = cv2.medianBlur(blur, int(blurs[0]))
        if int(blurs[1]) > 1:
            blur = cv2.GaussianBlur(blur, (int(blurs[1]), int(blurs[1])), 0)

    if threshold_type is not None:
        blur = pipeline.threshold(blur, **{'method': threshold_type})['img']

    if invert is not None:
        blur = ~blur

    if dilate_iterations is not None and dilate_iterations != 0:
        blur = cv2.dilate(blur, np.ones((5, 5), np.uint8), iterations=dilate_iterations)
    if erode_iterations is not None and erode_iterations != 0:
        blur = cv2.erode(blur, np.ones((5, 5), np.uint8), iterations=erode_iterations)

    if contour_type is not None:
        contours, _ = cv2.findContours(blur, cv2.RETR_LIST, contour_type)

        if cleanliness is not None and cleanliness > 0:
            arr = []
            for cont in contours:
                if cv2.arcLength(cont, False) > cleanliness:
                    arr.append(cont)
            contours = arr

        if smoothness is not None and smoothness > 0:
            arr = []
            for cont in contours:
                arr.append(cv2.approxPolyDP(cont, smoothness, False))
            contours = arr
        svg = pipeline.contours2svg(contours, width, height)

    add_img(blur, svg, idx)


def get_latest_img():
    for i in range(IMG_AMT)[::-1]:
        path = './staticFiles/images/pngs/' + str(i) + '.png'
        if os.path.exists(path):
            return i


def write_svg(svg, idx):
    with open('staticFiles/images/svgs/' + str(idx) + '.svg', 'w+') as f:
        f.write(svg)


def add_img(png, svg, idx):
    path = './staticFiles/images/pngs/'
    if idx < IMG_AMT - 1:
        cv2.imwrite(path + str(idx + 1) + '.png', png)
        write_svg(svg, idx + 1)
    else:
        for i in range(1, IMG_AMT):
            old_path = path + str(i) + '.png'
            new_path = path + str(i - 1) + '.png'
            if i == 1:
                os.remove(new_path)
                if os.path.exists('./staticFiles/images/svgs/0.svg'):
                    os.remove('./staticFiles/images/svgs/0.svg')
            os.rename(old_path, new_path)
            old_svg_path = './staticFiles/images/svgs/' + str(i) + '.svg'
            if os.path.exists(old_svg_path):
                os.rename(old_svg_path, './staticFiles/images/svgs/' + str(i - 1) + '.svg')
        cv2.imwrite(path + str(idx) + '.png', png)
        write_svg(svg, idx)


def undo():
    idx = get_latest_img()
    if idx > 0:
        os.remove('./staticFiles/images/pngs/' + str(idx) + '.png')
        svg_path = './staticFiles/images/svgs/' + str(idx) + '.svg'
        if os.path.exists(svg_path):
            os.remove(svg_path)


def delete_files_in_folder(path):
    all_files = os.listdir(path)
    for f in all_files:
        os.remove(path + f)


def restart():
    imgs = './staticFiles/images/'
    pngs = imgs + 'pngs/'
    svgs = imgs + 'svgs/'

    delete_files_in_folder(pngs)
    delete_files_in_folder(svgs)

    img = cv2.imread(imgs + 'backup.JPG')
    cv2.imwrite(pngs + '0.png', img)
    height, width = img.shape[:2]
    svg = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg"></svg>'
    write_svg(svg, 0)


if __name__ == '__main__':
    restart()
    app.run(host=config['host'], port=config['port'])
