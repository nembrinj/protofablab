from flask import Blueprint, render_template, request
from flask_login import login_required

from src import db
from src.models.user import User

requests_manager_bp = Blueprint("requests_manager", __name__)

from .forms import RequestsManagerForm


@requests_manager_bp.route("/requests_manager")
@login_required
def index():
  users = User.query.all()
  return render_template("requests_manager/index.html", users=users)


@requests_manager_bp.route("/requests_manager/enable")
@login_required
def Enable():
  user_id = request.args.get('id')
  print("enable called", user_id)
  
  user = User.query.get(user_id)
  user.is_enabled = True
  db.session.add(user)
  db.session.commit()
  
  return ""


@requests_manager_bp.route("/requests_manager/disable")
@login_required
def Disable():
  user_id = request.args.get('id')
  print("disable called", user_id)
  
  user = User.query.get(user_id)
  user.is_enabled = False
  db.session.add(user)
  db.session.commit()
  
  return ""