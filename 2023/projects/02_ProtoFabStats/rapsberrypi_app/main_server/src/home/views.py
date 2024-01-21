from flask import Blueprint, render_template, request
import os

home_bp = Blueprint("home", __name__)


@home_bp.route("/")
def index():
    host = str(os.popen("hostname -I | awk -F ' ' '{ print $1 }'").read().strip())
    return render_template("home/index.html", host=host)
