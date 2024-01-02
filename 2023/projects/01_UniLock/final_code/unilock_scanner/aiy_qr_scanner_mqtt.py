
import asyncio
import datetime
import time
from typing import Union


import cv2
import numpy as np

from pyzbar.pyzbar import decode
import paho.mqtt.client as mqtt

from aiy.leds import (Leds, Pattern, PrivacyLed, RgbLeds, Color)

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


def get_qr():

    cap = cv2.VideoCapture(0)
    # main flask server 
    qrcode = activate_camera(cap)
    
    print(qrcode)
    if qrcode: 
        return qrcode
    
    else:
        return "failed to get qr code"


client = mqtt.Client()

timeout = 30
mqttServer = "192.168.88.116" 
mqttPort = 1883
mqttUser = "mqttuser";
mqttPassword = "password";
mqttSubTopic = "activate"
mqttPubTopic = "qrcode"




def on_connect(client, userdata, flags, rc):
    client.subscribe(mqttSubTopic)

def on_message(client, userdata, msg):
    activation_message = str(msg.payload.decode("utf-8"))
    
    qrcode = get_qr()
    print(datetime.datetime.now(), qrcode)

    client.publish(mqttPubTopic, payload=qrcode, qos=0, retain=False)
print("activated")
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqttServer, mqttPort, 60)
print("listening")
client.loop_forever()

