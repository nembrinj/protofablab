

import paho.mqtt.client as mqtt
import random
import time

#mqttServer = "yourname.local"  # <------ CHANGE HERE
mqttServer = "localhost"  
mqttPort = 1883
mqttUser = "mqttuser";
mqttPassword = "password";
mqttTopic = "esp32/lux"

# see documentation from https://pypi.org/project/paho-mqtt/
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

client = mqtt.Client()
client.on_connect = on_connect

client.username_pw_set(mqttUser, mqttPassword)
client.connect(mqttServer, mqttPort, 60)

value = 500
msg_count = 0

while True:
    time.sleep(1)
    value += random.randint(-9, 9)
    result = client.publish(mqttTopic, "esp32 illuminance="+str(value))
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{value}` to topic `{mqttTopic}`")
    else:
        print(f"Failed to send message to topic {mqttTopic}")
    msg_count += 1
    if msg_count > 500:
        break

