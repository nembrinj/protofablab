###
### Iteration #3
###  - MQTT and WebSocket remains the same as previous versions
###  - Face Recognition is done using the MediaPipe platform, default settings
###
### Results
###  - FPS too low on a Pi4 + ESP32 setting
###

import cv2
import mediapipe as mp

import logging
import time
from datetime import datetime

import paho.mqtt.client as mqtt
from aiohttp import web
import aiohttp
import numpy as np
import io


# Debug
isVerbose = True

# MQTT
mqttServer = "192.168.15.54"
mqttPort = 1883
mqttTopic = "teddyCtrl"
mqttData = None

def on_publish(client, userdata, result):
    if(isVerbose): print("MQTT's publish ACK: client={}, result={}".format(client, result))
    pass

#
# Handle Websocket request and process image for face recognition
#
async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    async for msg in ws:
        if(isVerbose): print("---------------     New WebSocket Message:     ---------------")
        if msg.type == aiohttp.WSMsgType.TEXT:
            if msg.data == 'close':
                await ws.close()
        if msg.type == aiohttp.WSMsgType.BINARY:
            print("msg binary");

            # frame = cv2.VideoCapture(0)
            currentTime = time.time()
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
            print(frame)
            logging.info("Decode image from ESP: done, in {0:.2g} sec.".format(time.time()-currentTime))
            if(isVerbose):
                print("frame.shape: {}".format(frame.shape))

            fps_start = 0
            fps_total = 0
            img_count = 1

            with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face_detection:
                #while frame.isOpened():
                #success, image = frame.read()
                #if not success:
                #    print("Ignoring empty camera frame.")
                #    # If loading a video, use 'break' instead of 'continue'.
                #    continue

                fps_updated = time.time()
                fps = 1 / (fps_updated - fps_start)
                fps_start = fps_updated
                print(fps)

                # To improve performance, optionally mark the frame as not writeable to
                # pass by reference.
                frame.flags.writeable = False
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_detection.process(frame)

                # Draw the face detection annotations on the frame.
                frame.flags.writeable = True
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if results.detections:
                    for detection in results.detections:
                        mp_drawing.draw_detection(frame, detection)
                # Flip the frame horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Face Detection', cv2.flip(frame, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                    break

                img_count += 1
                fps_total += fps

            frame.release()
            print("avg FPS: {}".format(fps_total / img_count))
    
    print('websocket connection closed')    
    return ws


# LOGGING setup
currentTime = time.time()
root_logger = logging.getLogger()
root_logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('../log/mqtt-ws-mp_{:%d-%m-%Y-%H-%M-%S}.log'.format(datetime.now()), 'w', 'utf-8')
root_logger.addHandler(handler)

logging.info("[INFO] Logging initialized")

# FACE RECOGNITION:
# using MediaPipe: https://google.github.io/mediapipe/solutions/face_detection.html
logging.info("[INFO] Face Recognition: load MediaPipe...")
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

print("Mediapipe loaded")

# MQTT
logging.info("MQTT connecting to {}:{}...".format(mqttServer, mqttPort))
clientMQTT = mqtt.Client()
clientMQTT.on_publish =  on_publish
clientMQTT.connect(mqttServer, mqttPort)
logging.info("MQTT connected.")

print("MQTT DONE")

# WEBSOCKET
print("[INFO] WebSocket setup...", end="; ")
app = web.Application()
print("aiohttp application done", end="; ")
app.add_routes([web.get('/', websocket_handler)])
print("aiohttp routes added")
web.run_app(app)
