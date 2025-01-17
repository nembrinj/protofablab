
FROM ubuntu
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

WORKDIR /usr/src/app

RUN apt update

RUN apt-get install -y python3.8
RUN apt-get -y install python3-pip
RUN apt-get install ffmpeg libsm6 libxext6  -y
RUN pip install lapx --break-system-packages
RUN pip install opencv-python --break-system-packages
RUN pip install ultralytics --break-system-packages 
RUN pip install --no-cache-dir paho-mqtt --break-system-packages

# copy everything from this directory to WORKDIR
COPY . .

CMD [ "python3","-u", "./ML.py" ]
