from flask_wtf import FlaskForm
from src.models.user import User


class RequestsManagerForm(FlaskForm):
    print("RequestsManagerForm called")
