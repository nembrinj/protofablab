###
### Iteration #4: check basic again.
###  - MQTT and WebSocket
###  - OpenCV is used to show the webcam only, no other image processing
###
### Results
###  - FPS high at ~200-300 FPS
###

import logging
import time
from datetime import datetime

import aiohttp
from aiohttp import web
import paho.mqtt.client as mqtt

from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
import pickle
import cv2
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

def get_mqttData():
    return mqttData

def set_mqttData(msg_data):
    mqttData = msg_data

# Face Recognition
frameWidth = 0
depth = None
xPos = None

encodingsP = "models/encodings.pickle"  #Determine faces from encodings.pickle file model created from train_model.py

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
            #print("----type(...) ---- ")
            #print(type(msg))
            #print(type(msg.data))

            #print("---- .type --- ")
            #print(msg.type)
            #set_mqttData(msg.data)
            #! /usr/bin/python
            # import the necessary packages

            #Initialize 'currentname' to trigger only when a new person is identified.
            currentname = "unknown"

            # initialize the video stream and allow the camera sensor to warm up
            # Set the ser to the followng
            # src = 0 : for the build in single web cam, could be your laptop webcam
            # src = 2 : I had to set it to 2 inorder to use the USB webcam attached to my laptop
                
            #vs = VideoStream(src=2,framerate=10).start()
            #vs = VideoStream(usePiCamera=True).start()
            currentTime = time.time()
            frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
            logging.info("Decode image from ESP: done, in {0:.2g} sec.".format(time.time()-currentTime))
            if(isVerbose):
                print("frame.shape: {}".format(frame.shape))
            
            #cv_resized_img = cv2.resize(cv_img, (600, 800), interpolation = cv2.INTER_AREA)
            #dArray = np.asarray(bytearray(msg.data, dtype=np.uint8))
            
            #dArray = np.frombuffer(msg.data)
            #vs = dArray.reshape(600,-1)
            #time.sleep(2.0)

            # start the FPS counter
            fps = FPS().start()

            # loop over frames from the video file stream

            ## while True :
            # grab the frame from the threaded video stream and resize it
            # to 500px (to speedup processing)
            #frame = vs.read()
            #print("-----   frame  ------")
            #frame = cv_resized_img
            #print("frame: ", frame)
            #frame = imutils.resize(frame, width=500)
            #print("frame width")
            if(isVerbose):
                print("frame.shape[0]: {}".format(frame.shape[0]))

            currentTime = time.time()
            cv2.imshow("Facial Recognition is Running", frame)
            logging.info("show image: done, in {0:.2g} sec.".format(time.time()-currentTime))

            # display the image to our screen

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

#logging.basicConfig(format='%(asctime)s %(message)s',
#                    filename="log/raspberry4.log",
#                    encoding='utf-8',
#                    level=logging.DEBUG)

root_logger = logging.getLogger()
root_logger.setLevel(logging.DEBUG)
handler = logging.FileHandler('../log/raspberry4_{:%d-%m-%Y-%H-%M-%S}.log'.format(datetime.now()), 'w', 'utf-8')
root_logger.addHandler(handler)


#logging.info("[INFO] Logging initialized")

# FACE RECOGNITION:
# load the known faces and embeddings along with OpenCV's Haar cascade for face detection
#logging.info("[INFO] Face Recognition: loading encodings + face detector...")
#print("[INFO] Face Recognition: loading encodings + face detector...")
#data = pickle.loads(open(encodingsP, "rb").read())

#print("Pickle loaded")

# MQTT
logging.info("MQTT connecting to {}:{}...".format(mqttServer, mqttPort))
#print("[INFO] MQTT connecting to {}:{}...".format(mqttServer, mqttPort), end="")
clientMQTT = mqtt.Client()
clientMQTT.on_publish =  on_publish
clientMQTT.connect(mqttServer, mqttPort)
logging.info("MQTT connected.")
#print("MQTT connected.")

print("MQTT DONE")

# WEBSOCKET
print("[INFO] WebSocket setup...", end="; ")
app = web.Application()
print("aiohttp application done", end="; ")
app.add_routes([web.get('/', websocket_handler)])
print("aiohttp routes added")
web.run_app(app)

