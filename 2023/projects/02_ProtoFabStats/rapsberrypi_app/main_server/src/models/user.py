from datetime import datetime

from flask_login import UserMixin

from src import bcrypt, db


class User(UserMixin, db.Model):

    __tablename__ = "Users"

    id = db.Column(db.Integer, primary_key=True)
    email = db.Column(db.String, unique=True, nullable=False)
    password = db.Column(db.String, nullable=False)
    created_on = db.Column(db.DateTime, nullable=False)
    is_admin = db.Column(db.Boolean, nullable=False, default=False)
    is_enabled = db.Column(db.Boolean, nullable=False, default=False)


    def __init__(self, email, password, is_admin=False, is_enabled=False):
        self.email = email
        self.password = bcrypt.generate_password_hash(password)
        self.created_on = datetime.now()
        self.is_admin = is_admin
        self.is_enabled = is_enabled


    def __repr__(self):
        return f"<email {self.email}>"
