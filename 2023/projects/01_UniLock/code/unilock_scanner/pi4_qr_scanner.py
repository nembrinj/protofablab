
import asyncio
import datetime
import time
from typing import Union
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from flask import Flask

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

def activate_camera() -> Union[str, None]:
    """
    Activates the camera and LEDs, scans for a QR code, and returns the decoded data if found within the given timeout.

    Parameters:
    cap (cv2.VideoCapture): The camera capture object.

    Returns:
    Union[str, None]: The decoded QR code data as a string, or None if no QR code is found.
    """
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
