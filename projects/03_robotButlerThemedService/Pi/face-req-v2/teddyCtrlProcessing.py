###
### <INCOMPLETE WORK>
###
### Iteration #2: attempt to separate ../face-req-v1/  logic into two parts and use threads
###  - Face Recognition handled in ./img_processing.py
###  - Communication with MQTT and WebSocket as in ../face-req-v1/ 
###

import logging
import time

import aiohttp
from aiohttp import web
import paho.mqtt.client as mqtt

import imutils
import pickle
import time
import cv2
import numpy as np
import io

from img_processing import ImgProcessing

# Debug
isVerbose = True
firstInstance = True

# MQTT
mqttServer = "192.168.1.41"
mqttPort = 1883
mqttTopic = "teddyCtrl"
mqttOrder = "clientOrder"
mqttClientOrder = None
mqttData = None

def on_publish(client, userdata, result):
    if(isVerbose): print("MQTT's publish ACK: client={}, result={}".format(client, result))
    pass

def get_mqttData():
    return mqttData

def set_mqttData(msg_data):
    mqttData = msg_data

# Face Recognition
frameWidth = 0
depth = None
xPos = None

encodingsP = "encodings.pickle"  #Determine faces from encodings.pickle file model created from train_model.py

def get_frameWidth():
    return frameWidth

def set_frameWidth(ww):
    frameWidth = ww

def get_xPos():
    return xPos

def set_xPos(position):
    xPos = position

def get_depth():
    return depth

def set_depth(d):
    depth = d

#
# Handle Websocket request and process image for face recognition
#
async def websocket_handler(request):
    #frame_count = 0
    #recognition_count = 0

    ws = web.WebSocketResponse()
    await ws.prepare(request)

    async for msg in ws:
        if(isVerbose): print("---------------     New WebSocket Message:     ---------------")
        if msg.type == aiohttp.WSMsgType.TEXT:
            if msg.data == 'close':
                await ws.close()
        if msg.type == aiohttp.WSMsgType.BINARY:
            
            # Process Image using ImgProcessing instance
            faceRecognitionProcessing.start(msg.data)

            # mqtt
            currentTime = time.time()
            #clientMQTT.connect(mqttServer, mqttPort)
            
            mqttClientOrder = clientMQTT.subscribe(mqttOrder)
            print("MQTT Client Order: {}".format(mqttClientOrder))
            print("MQTT to be published: {}".format(faceRecognitionProcessing.get_stringJSON_data()))
            
            if(mqttClientOrder in faceRecognitionProcessing.get_names()):                
                mqtt_return = clientMQTT.publish(mqttTopic, faceRecognitionProcessing.get_stringJSON_data())
                logging.info("MQTT publish: done, in {0:.2g} sec.".format(time.time()-currentTime))
                if(isVerbose):
                    print("MQTT.publish return: {}".format(mqtt_return))
  
            # loop over the recognized faces
#             
#             currentTime = time.time()
#             for ((top, right, bottom, left), name) in zip(faceRecognitionProcessing.get_boxes(), faceRecognitionProcessing.get_names()):
#                 # draw the predicted face name on the image - color is in BGR
#                 cv2.rectangle(frame, (left, top), (right, bottom),
#                         (0, 255, 225), 2)
#                 y = top - 15 if top - 15 > 15 else top + 15
#                 cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
#                         .8, (0, 255, 255), 2)
#             logging.info("OpenCV draw rectanlges on image: done, in {0:.2g} sec.".format(time.time()-currentTime))

            # display the image to our screen
            cv2.imshow("Facial Recognition is Running", faceRecognitionProcessing.get_frame())
            #cv2.imwrite("dataset/recognitions/boxed_frame_%s.jpg" % recognition_count, frame)
            # recognition_count =  recognition_count + 1
            key = cv2.waitKey(1) & 0xFF

            # quit when 'q' key is pressed
            if key == ord("q"):
                # update the FPS counter
                fps.update()

                # stop the timer and display FPS information
                fps.stop()
                print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

                # do a bit of cleanup
                cv2.destroyAllWindows()
                #vs.stop()
                break

    print('websocket connection closed')
    
    return ws

# LOGGING setup
currentTime = time.time()
logging.basicConfig(format='%(asctime)s %(message)s',
#                     filename="log/raspberry4.log",
                    encoding='utf-8',
                    level=logging.DEBUG)
logging.info("[INFO] Face Recognition: loading encodings + face detector...")

# FACE RECOGNITION:
faceRecognitionProcessing = ImgProcessing()
# load the known faces and embeddings along with OpenCV's Haar cascade for face detection
logging.info("[INFO] Face Recognition: loading encodings + face detector...")
#print("[INFO] Face Recognition: loading encodings + face detector...")
data = pickle.loads(open(encodingsP, "rb").read())

# MQTT
logging.info("MQTT connecting to {}:{}...".format(mqttServer, mqttPort))
#print("[INFO] MQTT connecting to {}:{}...".format(mqttServer, mqttPort), end="")
clientMQTT = mqtt.Client()
clientMQTT.on_publish =  on_publish
clientMQTT.connect(mqttServer, mqttPort)
logging.info("MQTT connected.")
#print("MQTT connected.")

# WEBSOCKET
print("[INFO] WebSocket setup...", end="; ")
app = web.Application()
print("aiohttp application done", end="; ")
app.add_routes([web.get('/', websocket_handler)])
print("aiohttp routes added")
web.run_app(app)
