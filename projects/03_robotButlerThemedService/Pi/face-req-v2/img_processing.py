###
### <INCOMPLETE WORK>
###
### Iteration #2: attempt to separate ../face-req-v1/  logic into two parts and use threads
###  - Communication handled in ./teddyCtrlProcessing.py
###  - Face Recognition here is under a Class
###
###

import threading
import time
import cv2
import numpy as np
import logging
import face_recognition

from imutils.video import WebcamVideoStream

import mediapipe as mp

isVerbose = True

class ImgProcessing:
    def __init__(self):
        logging.basicConfig(format='%(asctime)s %(message)s',
                    #filename="log/imgProcessing.log",
                    encoding='utf-8',
                    level=logging.DEBUG)
        
        self.thread = None
        self.frame = None
        self.msgData = None
        self.is_running: bool = False
        self.stringJSON_data = None        # ...
        self.boxes = None                  # ...
        self.names = []                    # ...
        
    def start(self, msgData):
        self.msgData = msgData
        if self.thread is None:
            self.thread = threading.Thread(target=self._process)
            self.thread.start()            
            
    def get_names(self):
        return self.names
    
    def get_boxes(self):
        return self.boxes
    
    def get_stringJSON_data(self):
        return self.stringJSON_data
            
    def get_frame(self):
        return self.frame
    
    def stop(self):
        self.is_running = False
        self.thread.join()
        self.thread = None
        
    def _process(self):
        self.is_running = True
        while self.is_running:
            time.sleep(0.1)
            #Initialize 'currentname' to trigger only when a new person is identified.
            
            currentname = "unknown"
            currentTime = time.time()
            self.frame = cv2.imdecode(np.frombuffer(self.msgData, np.uint8), 1)
            logging.info("Decode image from ESP: done, in {0:.2g} sec.".format(time.time()-currentTime))
            if(isVerbose):
                print("frame.shape: {}".format(self.frame.shape))
            
            # start the FPS counter
            fps = FPS().start()

            # loop over frames from the video file stream
            if(isVerbose):
                print("frame.shape[0]: {}".format(self.frame.shape[0]))
            frameWidth = self.frame.shape[0]

            # Detect the face boxes
            currentTime = time.time()
            self.boxes = face_recognition.face_locations(self.frame)
            logging.info("Detect faces in image: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            # compute the facial embeddings for each face bounding box
            currentTime = time.time()
            encodings = face_recognition.face_encodings(self.frame, self.boxes)
            logging.info("Get facial encodings for each face: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            self.names = []

            currentTime = time.time()
            face_landmarks_list = face_recognition.face_landmarks(self.frame)
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
                cv2.line(self.frame, midPointL,midPointR , (0,200,0),3)
                
                # draw eyes
                cv2.circle(self.frame, midPointL, 5, (255,0,255), cv2.FILLED)
                cv2.circle(self.frame, midPointR, 5, (255,0,255), cv2.FILLED)
                
                #calculate distance between 2 eyes. In reality 64mm for men, 62 for women in average.
                # Won't change in real life but in the frame, this is based of calculus
                w = float(cv2.magnitude(midPointL,midPointR)[0])
                W = 6.3 #average from men to women eye distance
                
                # put fixed distance and placed a face in front in order to find the focal length, 
                # once computed, put f as cst and remove d and print
                #d = 50
                #print("focal length")
                #print((d*w)/W)
                f = 5176.532165530412

                # will calculate depth
                d = (f*W)/w
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
            self.stringJSON_data = "{"
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
                
                self.stringJSON_data += "\""+ str(indexPerson) + "\":{\"xPos\":" + str(int(xPosList[indexPerson])) + ",\"depth\":" + str(int(depthList[indexPerson])) + ",\"name\":\"" + currentname + "\"},"
                indexPerson += 1
                
                # update the list of names
                self.names.append(name)
            
            logging.info("Compare face encodings vs candidate for a match: done, in {0:.2g} sec.".format(time.time()-currentTime))
            
            # remove the last coma en ends JSON
            self.stringJSON_data = self.stringJSON_data[:-1]
            self.stringJSON_data += "}"
    
    #
    # Detect the face boxes:
    #
    def detect_face_boxes(self):
        self.boxes = face_recognition.face_locations(self.frame, model="hog")


###
### Basic use of CV2 for face detection
### results: very slow, even in grayscale
###

def setup_basic_face_boxe():
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def basic_face_boxe(img_raw):
    # source: https://github.com/adarsh1021/facedetection/blob/master/detect_face_image.py
    img_gray = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(img_gray, 1.1, 4)
    for (x, y, w, h) in faces:
        cv2.rectangle(img_raw, (x, y), (x + w, y + h), (255, 0, 0), 2)

    return img_raw


if __name__ == "__main__":
    ###
    ### USING MEDIA PIPE: https://google.github.io/mediapipe/solutions/face_detection.html
    ###
    mp_face_detection = mp.solutions.face_detection
    mp_drawing = mp.solutions.drawing_utils
    cap = cv2.VideoCapture(0)

    fps_start = 0
    fps_total = 0
    img_count = 0

    with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face_detection:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                # If loading a video, use 'break' instead of 'continue'.
                continue


            fps_updated = time.time()
            fps = 1 / (fps_updated - fps_start)
            fps_start = fps_updated
            print(fps)

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = face_detection.process(image)

            # Draw the face detection annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.detections:
                for detection in results.detections:
                    mp_drawing.draw_detection(image, detection)
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Face Detection', cv2.flip(image, 1))
            if cv2.waitKey(5) & 0xFF == 27:
                break

            img_count += 1
            fps_total += fps

    cap.release()
    print("avg FPS: {}".format(fps_total / img_count))


# if __name__ == "__main__":
#     print("---img_processing.py---")

#     cap =  WebcamVideoStream(src=0, name="WebcamVideoStream")
#     cap.start()

#     ImgProcessing = ImgProcessing()

#     fps_start = 0
#     fps_total = 0
#     img_count = 0

#     while True:
#         img_raw = cap.read()
#         img_raw.flags.writeable = False   # improve performance
#         ImgProcessing.frame = img_raw

#         ImgProcessing.detect_face_boxes()

#         fps_updated = time.time()
#         fps = 1 / (fps_updated - fps_start)
#         fps_start = fps_updated
#         print(fps)

#         cv2.putText(img_raw,
#                     f'FPS: {int(fps)}',
#                     (20, 70),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     1,
#                     (0, 255, 0),
#                     2
#         )

#         img_count += 1
#         fps_total += fps

#         cv2.waitKey(1)

#         cv2.imshow("Image", img_raw)
#         if cv2.waitKey(1) == ord('q'):
#             break

#     cap.stop()
#     print("avg FPS: {}".format(fps_total / img_count))