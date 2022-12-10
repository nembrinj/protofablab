import json

import cv2
import numpy as np
import toml
from flask import Flask, request

import pipeline.pipeline as pipeline

app = Flask(__name__)
config = toml.load('../config.toml')['Server']

with open('../pipeline/pipeline.json') as f:
    typ = dict[str, str | dict[str, str | list[str] | dict[str, int | float | str]]]
    pipeline_json: typ = json.load(f)

print(pipeline_json)

for method, body in pipeline_json.items():
    print(method, body)


# TODO: Maybe this doesn't work correctly, I couldn't try yet with a frontend
@app.route('/', methods=['Post'])
def root():
    file = request.files['image']
    img_buf = np.asarray(bytearray(file.stream.read()), dtype="uint8")
    img = cv2.imdecode(img_buf, cv2.IMREAD_UNCHANGED)

    args = dict(request.form.deepcopy())
    args['img'] = img

    return pipeline.call_pipeline(args)

# TODO: Add actual routes when needed


if __name__ == '__main__':
    app.run(host=config['host'], port=config['port'])
