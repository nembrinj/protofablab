import json
import cv2
import numpy as np
import os
import toml
from flask import Flask, flash, request, render_template, redirect, url_for
from werkzeug.utils import secure_filename

import pipeline.pipeline as pipeline

UPLOAD_FOLDER = './staticFiles/images'
ALLOWED_EXTENSIONS = {'png', 'jpg', 'jpeg'}

app = Flask(__name__, template_folder='templateFiles', static_folder='staticFiles')
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['SECRET_KEY'] = 'the random string'
config = toml.load('../config.toml')['Server']

with open('../pipeline/pipeline.json') as f:
    typ = dict[str, str | dict[str, str | list[str] | dict[str, int | float | str]]]
    pipeline_json: typ = json.load(f)


def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS


# TODO: Maybe this doesn't work correctly, I couldn't try yet with a frontend
@app.route('/', methods=['GET', 'POST'])
def upload_file():
    if request.method == 'POST':
        if request.form.getlist('undo'):
            copy_backup()
            return redirect(request.url)
        blurs = request.form.getlist('blur')
        threshold = request.form.getlist('threshold')
        contour = request.form.getlist('contour')
        smooth = request.form.getlist('smooth')
        di_er = request.form.getlist('dier')

        # check if the post request has the file part
        if 'file' not in request.files:
            flash('No file part')
            root(blurs, threshold, contour, smooth, di_er)
            return redirect(request.url)
        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if file.filename == '':
            flash('No selected file')
            root(blurs, threshold, contour, smooth, di_er)
            return redirect(request.url)
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(os.path.join(app.config['UPLOAD_FOLDER'], 'main.png'))
            root(blurs, threshold, contour, smooth, di_er)
            return redirect(request.url)
    return render_template('index.html')


def dilate_erode(blur, dier):
    blur = cv2.dilate(blur, np.ones((5, 5), np.uint8), iterations=int(dier[0]))
    blur = cv2.erode(blur, np.ones((5, 5), np.uint8), iterations=int(dier[1]))
    return blur


def root(blurs, threshold, contour, smooth, di_er):
    img = cv2.imread('./staticFiles/images/main.png')
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur = gray

    if 'median' in blurs:
        blur = cv2.medianBlur(blur, 3)
    if 'gauss' in blurs:
        blur = cv2.GaussianBlur(blur, (5, 5), 0)

    if threshold:
        blur = pipeline.threshold(blur, **{'method': threshold[0]})['img']

    if di_er != ['0', '0']:
        blur = dilate_erode(blur, di_er)

    if contour:
        contours, _ = cv2.findContours(blur, cv2.RETR_LIST, int(contour[0]))

        if int(smooth[0]) > 0:
            arr = []
            for cont in contours:
                if cv2.arcLength(cont, False) > 5:
                    arr.append(cv2.approxPolyDP(cont, int(smooth[0]), False))
            contours = arr

        n, m = gray.shape
        f = open('staticFiles/images/main.svg', 'w+')
        f.write('<svg width="' + str(m) + '" height="' + str(n) + '" xmlns="http://www.w3.org/2000/svg">')
        for c in contours:
            f.write('<path d="M')
            for cords in c:
                x, y = cords[0]
                f.write(str(x) + ' ' + str(y) + ' ')
            f.write('"/>')
        f.write('</svg>')
        f.close()

    cv2.imwrite('./staticFiles/images/main.png', blur)


def copy_backup():
    img = cv2.imread('./staticFiles/images/backup.JPG')
    cv2.imwrite('./staticFiles/images/main.png', img)
    if os.path.exists('./staticFiles/images/main.svg'):
        os.remove('./staticFiles/images/main.svg')


# TODO: Add actual routes when needed


if __name__ == '__main__':
    copy_backup()
    app.run(host=config['host'], port=config['port'])
