from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from flask_mqtt import Mqtt
#from flask_influxdb import InfluxDB
import time
import json



from datetime import datetime

app = Flask(__name__)
app.config.from_pyfile("config.cfg")
socketio = SocketIO(app)
#influx_db = InfluxDB(app=app)

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
        luxValue = val
    )
    toSend = json.dumps(toSend)
    socketio.send(toSend)



@app.route('/')
def index():
    """current lux reading page"""
    return render_template('index.html')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, use_reloader=False, debug=True)
