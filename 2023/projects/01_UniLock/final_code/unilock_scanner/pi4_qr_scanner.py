
import asyncio
import datetime
import time
from typing import Union


import cv2
import numpy as np

from pyzbar.pyzbar import decode
from flask import Flask




timeout = 60

def decoder(image) -> Union[str, None]:

    # input : image captured from the webcam
    # output : either a string extrated from a qr code or None if no qrcode was detected 
    # use pyzbar to extra the infromation from the qrcode 

    gray_img = cv2.cvtColor(image,0)
    qrcode = decode(gray_img)
    
    if len(qrcode) != 0:
        obj = qrcode[0]
        points = obj.polygon
        (x,y,w,h) = obj.rect
        pts = np.array(points, np.int32)
        pts = pts.reshape((-1, 1, 2))

        qrcodeData = obj.data.decode("utf-8")
        qrcodeType = obj.type
    
        return str(qrcodeData)
    
    return None

def activate_camera() -> Union[str, None]:

    # output : either a string extrated from a qr code or None if no qrcode was detected by the timeout
    # controls the leds and camera attachted to rasberry pi 
    # runs for how many seconds until the timeout is met 


    endTime = datetime.datetime.now() + datetime.timedelta(seconds=timeout)
    cap = cv2.VideoCapture(0)
    while datetime.datetime.now() <= endTime :
        ret, frame = cap.read()
        qrcode = decoder(frame)
        if qrcode:
            print("success : ", qrcode)
            time.sleep(1)
            return qrcode

        print("failed")
        time.sleep(1)
        return None

app = Flask(__name__)

@app.route('/')
def main():

    # main flask server 

    qrcode = activate_camera()
    
    if qrcode: 

        return qrcode
    
    else:
        return "failed to get qr code"
