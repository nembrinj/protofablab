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
            frameWidth = frame.shape[0]

            #print("---------------------------")
            # save image
            #cv2.imwrite("dataset/frames/image_converted_%s.jpg" % frame_count, frame)
            #frame_count = frame_count + 1


            # Detect the face boxes
            currentTime = time.time()
            boxes = face_recognition.face_locations(frame)
            logging.info("Detect faces in image: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            # compute the facial embeddings for each face bounding box
            currentTime = time.time()
            encodings = face_recognition.face_encodings(frame, boxes)
            logging.info("Get facial encodings for each face: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            names = []

            currentTime = time.time()
            face_landmarks_list = face_recognition.face_landmarks(frame)
            logging.info("Get Face feature locations for each images: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            # get eyes
            depthList = []
            xPosList = []
            currentTime = time.time()
            for face_landmark in face_landmarks_list:
                # have the points on eyes
                left_eye = np.array(face_landmark['left_eye'])
                right_eye = np.array(face_landmark['right_eye'])
                nose_bridge = np.array(face_landmark['nose_bridge'])
                nose_tip = np.array(face_landmark['nose_tip'])
                if(isVerbose):
                    print("left eye: {}".format(left_eye))
                
                ## coordinates for eyes https://cdn-images-1.medium.com/max/1600/1*AbEg31EgkbXSQehuNJBlWg.png
                # will take the midpoint of 2 eye's corner as center of eye for the offset between eyes.
                # corner 1 at index 0, corner 2 at index 3
                midPoint_left_eye = ((left_eye[0][0]+left_eye[3][0])/2, (left_eye[0][1]+left_eye[3][1])/2)
                midPoint_right_eye = ((right_eye[0][0]+right_eye[3][0])/2, (right_eye[0][1]+right_eye[3][1])/2)
                
                if(isVerbose):
                    print("mid point left_eye: {}".format(midPoint_left_eye))
                    print("mid point right_eye: {}".format(midPoint_right_eye))
                
                ## calculate averaging for nose that will be considered as center of the face. NB, only x is taken in account, no need to have Y centered
                # to have as if the axis was in the middle, take frame width and subtract the position of nose
                sumPosXY = nose_bridge.sum(axis = 0) + nose_tip[0] + nose_tip[2] + nose_tip[4]
                totalElement = nose_bridge.shape[0] + 3
                if(isVerbose):
                    print("sumPosXY: {}".format(sumPosXY))
                    
                    print("totalElement: {}".format(totalElement))
                
                    print("frameWidth: {}".format(frameWidth))

                xPos = frameWidth - sumPosXY[0]/totalElement
                if(isVerbose):
                    print("xPos: {}".format(xPos))
                
                xPosList.append(xPos)
                if(isVerbose):
                    print("xPosList: {}".format(xPosList))
                
                midPointL = (int(midPoint_left_eye[0]),int(midPoint_left_eye[1]))
                midPointR = (int(midPoint_right_eye[0]),int(midPoint_right_eye[1]))

                # draw line between the eyes.
                cv2.line(frame, midPointL,midPointR , (0,200,0),3)
                
                # draw eyes
                cv2.circle(frame, midPointL, 5, (255,0,255), cv2.FILLED)
                cv2.circle(frame, midPointR, 5, (255,0,255), cv2.FILLED)
                
                #calculate distance between 2 eyes. In reality 64mm for men, 62 for women in average.
                # Won't change in real life but in the frame, this is based of calculus
                w = float(cv2.magnitude(midPointL,midPointR)[0])
                W = 6.3 #average from men to women eye distance
                
                # put fixed distance and placed a face in front in order to find the focal length, 
                # once computed, put f as cst and remove d and print
                #d = 50
                #print("focal length")
                #print((d*w)/(W+w))
                f = 5176.532165530412

                # will calculate depth
                d = (f*(W+w))/w
                depthList.append(d)
                if(isVerbose):
                    print("depthList: {}".format(depthList))
            
            logging.info("FaceLandmark: done, in {0:.2g} sec.".format(time.time()-currentTime))
            # outside the loop because list of depths and position for every faces in the frame
            #set_depth(d)
            #set_xPos(xPosList)
            
            # variables to create a JSON while looping over recognized face.
            # as we don't want to have multiple time the same face, we created an array to know if it's in the JSON or not.
            # index as counter to access person's information
            stringJSON_data = "{"
            indexPerson = 0

            # loop over the facial embeddings
            currentTime = time.time()
            for encoding in encodings:
                # attempt to match each face in the input image to our known
                # encodings
                matches = face_recognition.compare_faces(data["encodings"],
                        encoding)
                name = "Unknown" #if face is not recognized, then print Unknown

                # check to see if we have found a match
                if True in matches:
                    # find the indexes of all matched faces then initialize a
                    # dictionary to count the total number of times each face
                    # was matched
                    matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                    counts = {}

                    # loop over the matched indexes and maintain a count for
                    # each recognized face face
                    for i in matchedIdxs:
                        name = data["names"][i]
                        counts[name] = counts.get(name, 0) + 1
                            
                    # determine the recognized face with the largest number
                    # of votes (note: in the event of an unlikely tie Python
                    # will select first entry in the dictionary)
                    name = max(counts, key=counts.get)
                    

                    #If someone in your dataset is identified, print their name on the screen
                    if currentname != name:
                        currentname = name
                        if(isVerbose): print(currentname)
                

                if(isVerbose):
                    print("indexPerson: {}".format(indexPerson))
                    
                    print("xPosList[indexPerson]: {}".format(xPosList[indexPerson]))
                    
                    print("depthList[indexPerson]: {}".format(depthList[indexPerson]))
                    
                    print("currentname: {}".format(currentname))
                
                stringJSON_data += "\""+ str(indexPerson) + "\":{\"xPos\":" + str(int(xPosList[indexPerson])) + ",\"depth\":" + str(int(depthList[indexPerson])) + ",\"name\":\"" + currentname + "\"},"
                indexPerson += 1
                
                # update the list of names
                names.append(name)
            
            logging.info("Compare face encodings vs candidate for a match: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            # remove the last coma en ends JSON
            stringJSON_data = stringJSON_data[:-1]
            stringJSON_data += "}"

            # mqtt
            currentTime = time.time()
            #clientMQTT.connect(mqttServer, mqttPort)
            mqtt_return = clientMQTT.publish(mqttTopic, stringJSON_data)
            logging.info("MQTT publish: done, in {0:.2g} sec.".format(time.time()-currentTime))
            if(isVerbose):
                print("MQTT.publish return: {}".format(mqtt_return))
  
            # loop over the recognized faces
            currentTime = time.time()
            for ((top, right, bottom, left), name) in zip(boxes, names):
                # draw the predicted face name on the image - color is in BGR
                cv2.rectangle(frame, (left, top), (right, bottom),
                        (0, 255, 225), 2)
                y = top - 15 if top - 15 > 15 else top + 15
                cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                        .8, (0, 255, 255), 2)
            logging.info("OpenCV draw rectanlges on image: done, in {0:.2g} sec.".format(time.time()-currentTime))

            # display the image to our screen
            cv2.imshow("Facial Recognition is Running", frame)
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
handler = logging.FileHandler('log/raspberry4_{:%d-%m-%Y-%H-%M-%S}.log'.format(datetime.now()), 'w', 'utf-8')
root_logger.addHandler(handler)


logging.info("[INFO] Logging initialized")

# FACE RECOGNITION:
# load the known faces and embeddings along with OpenCV's Haar cascade for face detection
logging.info("[INFO] Face Recognition: loading encodings + face detector...")
#print("[INFO] Face Recognition: loading encodings + face detector...")
data = pickle.loads(open(encodingsP, "rb").read())

print("Pickle loaded")

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

