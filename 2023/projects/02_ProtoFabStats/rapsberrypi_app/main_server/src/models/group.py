from src import db


class Group(db.Model):

    __tablename__ = "Groups"

    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String, unique=True, nullable=False)


    def __init__(self, name):
        self.name = name


    def __repr__(self):
        return f"<name {self.name}>"
