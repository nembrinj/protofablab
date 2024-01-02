import paho.mqtt.client as mqtt
mqttServer = "localhost" # <------ CHANGE HERE
mqttPort = 1883
mqttUser = "mqttuser";
mqttPassword = "password";
mqttTopic = "activate"
def on_connect(client, userdata, flags, rc):
    client.subscribe(mqttTopic)
def on_message(client, userdata, msg):
    lux = str(msg.payload.decode("utf-8"))
    print(lux)
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqttServer, mqttPort, 60)
client.loop_forever()
