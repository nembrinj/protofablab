
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
    """
    Decodes QR code from the given image captured from the webcam using the ZBar library.

    Parameters:
    image (numpy.ndarray): The input image containing the QR code.

    Returns:
    Union[str, None]: The decoded QR code data as a string, or None if no QR code is found.
    """
    gray_img = cv2.cvtColor(image,0)
    qrcode = decode(gray_img)
    
    if len(qrcode) != 0:
        obj = qrcode[0]
        
        qrcodeData = obj.data.decode("utf-8") 
        return str(qrcodeData)
    
    return None

def activate_camera(cap) -> Union[str, None]:
    """
    Activates the camera and LEDs, scans for a QR code, and returns the decoded data if found within the given timeout.

    Parameters:
    cap (cv2.VideoCapture): The camera capture object.

    Returns:
    Union[str, None]: The decoded QR code data as a string, or None if no QR code is found.
    """
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
    # main Flask server 
    qrcode = activate_camera(cap)
    print(qrcode)
    if qrcode: 
        return qrcode
    
    else:
        return "failed to get qr code"
