FROM arm32v6/python:3.9.14-alpine3.15

WORKDIR /usr/src/main_server

COPY . .

RUN pip install -r requirements.txt

RUN source .env

RUN flask db init
RUN flask db migrate
RUN flask db upgrade

EXPOSE 5000

RUN python manage.py run

