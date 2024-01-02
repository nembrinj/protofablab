Within the project directory, create a `.env` with the following content:
```txt
REAL_EXTERNAL_API_ENDPOINT=FALSE 
REAL_CAMERA_API_ENDPOINT=TRUE
EXTERNAL_PROJECT_API_ENDPOINT = "filler"
CAMERA_API_ENDPOINT = "http://<ip/hostname of the scanner>:5000"

MQTT_SERVER_HOST=localhost
MQTT_SERVER_PORT=1883
```

Create docker image:
```bash
docker build -t unilock_app_image .
```

Instantiate container from docker image:
```bash
docker-compose up -d
```


Enjoy your application on http://localhost:8000.