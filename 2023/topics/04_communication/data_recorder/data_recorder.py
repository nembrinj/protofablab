
import time
import csv
from pathlib import Path

import paho.mqtt.client as mqtt

mqttServer = "steve.local"  # <------ CHANGE HERE
mqttPort = 1883
mqttUser = "mqttuser";
mqttPassword = "password";
mqttTopic = "TEST"

# see documentation from https://pypi.org/project/paho-mqtt/
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(mqttTopic)

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    lux = str(msg.payload.decode("utf-8"))

    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
    print(timestr)

    columns = ["datetime", "lux"]

    FILE_PATH = Path('data/illuminance.csv')

    if not FILE_PATH.exists():
        with open(FILE_PATH, 'w', newline='') as csv_file:
            csv_file_writer = csv.writer(csv_file)
            csv_file_writer.writerow(columns)

    with open(FILE_PATH, 'a', newline='') as csv_file:
        csv_file_append = csv.writer(csv_file)
        csv_file_append.writerow([timestr,lux])
    

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set(mqttUser, mqttPassword)
client.connect(mqttServer, mqttPort, 60)

client.loop_forever()

