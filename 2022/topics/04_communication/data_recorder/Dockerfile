#FROM python:3
FROM arm32v6/python:3.9.14-alpine3.15

WORKDIR /usr/src/app

RUN mkdir data

RUN pip install --no-cache-dir requests 
RUN pip install --no-cache-dir paho-mqtt 

COPY . .

CMD [ "python", "./data_recorder.py" ]
