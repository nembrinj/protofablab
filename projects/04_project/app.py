from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from flask_mqtt import Mqtt
import time
import json



from datetime import datetime

app = Flask(__name__)
socketio = SocketIO(app)
app.config['SECRET_KEY'] = 'secret!'
app.config['MQTT_BROKER_URL'] = 'maqiatto.com'
app.config['MQTT_BROKER_PORT'] = 1883
app.config['MQTT_USERNAME'] = 'mathias.tonini@gmail.com'  # Set this item when you need to verify username and password
app.config['MQTT_PASSWORD'] = '123456789'  # Set this item when you need to verify username and password
app.config['MQTT_KEEPALIVE'] = 15  # Set KeepAlive time in seconds
app.config['MQTT_TLS_ENABLED'] = False  # If your server supports TLS, set it True
topic = 'mathias.tonini@gmail.com/protofablab/lux'
luxValue = 0;
mqtt_client = Mqtt(app)

@mqtt_client.on_connect()
def handle_connect(client, userdata, flags, rc):
   if rc == 0:
       print('Connected successfully')
       mqtt_client.subscribe(topic) # subscribe topic
   else:
       print('Bad connection. Code:', rc)

@mqtt_client.on_message()
def handle_mqtt_message(client, userdata, message):
    data = dict(
        topic=message.topic,
        payload=message.payload.decode()
    )
    if data["topic"] == "mathias.tonini@gmail.com/protofablab/lux":
        print(data["payload"])
        handle_message(data["payload"])

@socketio.on('message')
def handle_message(val):
    toSend = dict(
        timestamp = (time.time_ns()) // 1000000,
        message = val
    )
    toSend = json.dumps(toSend)
    socketio.send(toSend)



@app.route('/')
def index():
    """current lux reading page"""
    return render_template('index.html')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, use_reloader=False, debug=True)
