import http
import os
import random
import time
from logging import critical

from flask import Flask, request

if "API_USER" not in os.environ or "API_PASS" not in os.environ:
    critical("environment variables API_USER and API_PASS must be defined.")
    exit(1)

app = Flask(__name__)


@app.before_request
def check_auth():
    # check for basic authentication
    if request.method == "POST":
        auth = request.authorization
        if not auth or auth.username != os.environ["API_USER"] or auth.password != os.environ["API_PASS"]:
            return "unauthorized", http.HTTPStatus.UNAUTHORIZED


@app.route("/")
def index():
    return "<h3>Endpoints:</h3>" \
           "<ul>" \
           "<li>POST /print</li>" \
           "<li>POST /door</li>" \
           "</ul>"


@app.route("/print", methods=["POST"])
def print_document():
    # print provided document
    time.sleep(3)

    # assume 80% chance of success
    success = random.uniform(0, 1) < 0.8

    if success:
        return "document printed successfully", http.HTTPStatus.OK
    else:
        return "failed to print document", http.HTTPStatus.INTERNAL_SERVER_ERROR


@app.route("/door", methods=["POST"])
def lock_door():
    if request.content_type != "application/json":
        return "body must be in JSON format", http.HTTPStatus.BAD_REQUEST

    if "action" not in request.json.keys():
        return "missing field 'action'", http.HTTPStatus.BAD_REQUEST

    action = request.json["action"]

    if action not in ["lock", "unlock"]:
        return f"invalid action: '{action}'", http.HTTPStatus.BAD_REQUEST

    # lock or unlock door
    time.sleep(1)

    # assume 90% chance of success
    success = random.uniform(0, 1) < 0.9

    if success:
        return f"door {action}ed successfully", http.HTTPStatus.OK
    else:
        return f"failed to {action} door", http.HTTPStatus.INTERNAL_SERVER_ERROR


if __name__ == "__main__":
    app.run(host="0.0.0.0")
