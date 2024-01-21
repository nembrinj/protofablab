import getpass
from random import random

from flask.cli import FlaskGroup

from src import app, db
from src.models.user import User

cli = FlaskGroup(app)


def _create_user(is_enabled):
    email = input("Enter email address: ")
    password = getpass.getpass("Enter password: ")
    confirm_password = getpass.getpass("Enter password again: ")
    if password != confirm_password:
        print("Passwords don't match")
        return

    try:
        user = User(email=email, password=password, is_admin=False, is_enabled=is_enabled)
        db.session.add(user)
        db.session.commit()
        print(f"User with email {email} created successfully!")
    except Exception:
        print("Couldn't create user.")


@cli.command("create_admin")
def create_admin():
    """Creates the admin user."""
    email = input("Enter email address: ")
    password = getpass.getpass("Enter password: ")
    confirm_password = getpass.getpass("Enter password again: ")
    if password != confirm_password:
        print("Passwords don't match")
        return

    try:
        user = User(email=email, password=password, is_admin=True, is_enabled=True)
        db.session.add(user)
        db.session.commit()
        print(f"Admin with email {email} created successfully!")
    except Exception:
        print("Couldn't create admin user.")


@cli.command("create_enabled_user")
def create_enabled_user():
    _create_user(is_enabled=True)


@cli.command("create_disabled_user")
def create_disabled_user():
    _create_user(is_enabled=False)


@cli.command("create_dummy_users")
def create_dummy_users():
    """Creates 10 dummy users."""
    try:
        for index in range(10):
            email = f"dummy_user_{index}@unifr.ch"
            password = "password"
            is_enabled = bool(random() % 2)
            user = User(email=email, password=password, is_enabled=is_enabled)
            db.session.add(user)
        db.session.commit()
        print("Dummy users created.")
    except Exception:
        print("Couldn't create dummy users.")


if __name__ == "__main__":
    cli()