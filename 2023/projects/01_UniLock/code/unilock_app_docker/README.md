### Web Application

First, clone the github repository:
```bash
   git clone https://github.com/nembrinj/protofablab.git
```

Move to the UniLock project directory:
```bash
cd 2023/projects/01_UniLock
```

Create a `.env` file in the UniLock root directory with the following content:
```txt
REAL_EXTERNAL_API_ENDPOINT=FALSE 
REAL_CAMERA_API_ENDPOINT=TRUE
EXTERNAL_PROJECT_API_ENDPOINT = "replace when needed"
CAMERA_API_ENDPOINT = "http://<ip/hostname of the scanner>:5000"
MQTT_SERVER_HOST=localhost
MQTT_SERVER_PORT=1883
```

Navigate to the `unilock_app_docker` directory:
```
cd code/unilock_app_docker
```

Create a docker image:
```bash
docker build -t unilock_app_image .
```

Instantiate a container from the docker image:
```bash
docker-compose up -d
```

Enjoy your application on [http://localhost:8000](http://localhost:8000).