
import asyncio
import datetime
import time
from typing import Union


import cv2
import numpy as np

from pyzbar.pyzbar import decode
from flask import Flask


from aiy.leds import (Leds, Pattern, PrivacyLed, RgbLeds, Color)


timeout = 30

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

def activate_camera(cap) -> Union[str, None]:

    # output : either a string extrated from a qr code or None if no qrcode was detected by the timeout
    # controls the leds and camera attachted to rasberry pi 
    # runs for how many seconds until the timeout is met 
    with Leds() as leds:

        endTime = datetime.datetime.now() + datetime.timedelta(seconds=timeout)
        leds.update(Leds.rgb_on(Color.YELLOW)) 
        while datetime.datetime.now() <= endTime :
            ret, frame = cap.read()
            qrcode = decoder(frame)
            if qrcode:
                print("success : ", qrcode)
                leds.update(Leds.rgb_on(Color.GREEN))
                time.sleep(1)
                return qrcode

        leds.update(Leds.rgb_on(Color.RED))
        print("failed")
        time.sleep(1)
        leds.update(Leds.rgb_on(Color.WHITE))
        return None

app = Flask(__name__)

@app.route('/turn_on')
def main():

    cap = cv2.VideoCapture(0)
    # main flask server 
    qrcode = activate_camera(cap)
    
    print(qrcode)
    if qrcode: 
        return qrcode
    
    else:
        return "failed to get qr code"
