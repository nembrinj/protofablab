
FROM ubuntu
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /usr/src/app

RUN apt update

RUN apt-get install -y python3.8
RUN apt-get -y install python3-pip
RUN pip install --no-cache-dir pyserial --break-system-packages
RUN pip install --no-cache-dir paho-mqtt --break-system-packages

# copy everything from this directory to WORKDIR
COPY . .

CMD [ "python3","-u", "./mqtt_arduino.py" ]
