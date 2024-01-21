# Main Server

Helpful tutorial for setting up Basic User Authentication in a Flask App: https://github.com/ashutoshkrris/Flask-User-Authentication/tree/main.

## How to Run

We used a virtual environment (named *main_server_venv* in these intstructions). All required python libraries are in the *requirements.txt* file. They can be installed using the command `pip install -r requirements.txt`.

Before starting the server, the database has to be initialized using the following set of commands:
```
flask db init
flask db migrate
flask db upgrade
```

Now we can export the environment variables fron *.env* and start the server. Make sure to do that from the virtual environment.
```
source main_server_venv/bin/activate
source .env
python manage.py run
```

The server will be running on http://127.0.0.1:5000.
