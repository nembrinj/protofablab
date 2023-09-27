from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import socket
import mariadb
from influxdb import InfluxDBClient
import time
import json
import _thread
from datetime import datetime
import urllib.parse

app = Flask(__name__)
app.config.from_pyfile("config.py",silent=False)
socketio = SocketIO(app,cors_allowed_origins="*")
luxValue = 0;

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
localIp = (s.getsockname()[0])
print("local IP: "+localIp)

def getTemp():
    r = client.query("SELECT * from temperature GROUP BY * order by DESC LIMIT 1")
    #print(list(r.get_points(measurement="temperature"))[1]["value"])
    return list(r.get_points(measurement="temperature"))[1]["value"]

def getLux():
    r = client.query("SELECT * from lux GROUP BY * order by DESC LIMIT 1")
    return list(r.get_points(measurement="lux"))[0]["value"]

def getCo2():
    r = client.query("SELECT * from co2 GROUP BY * order by DESC LIMIT 1")
    return list(r.get_points(measurement="co2"))[0]["value"]

def getPressure():
    r = client.query("SELECT * from tvoc GROUP BY * order by DESC LIMIT 1")
    return list(r.get_points(measurement="tvoc"))[0]["value"]

def getHumidity():
    r = client.query("SELECT * from humidity GROUP BY * order by DESC LIMIT 1")
    return list(r.get_points(measurement="humidity"))[0]["value"]


@socketio.on('message')
def handle_message():
    toSend = dict(
        temperature = getTemp(),
        lux = getLux(),
        c02 = getCo2(),
        pressure = getPressure(),
        humidity = getHumidity(),

    )
    toSend = json.dumps(toSend)
    socketio.send(toSend)

@app.route('/')
def index():
    return render_template('index.html',username='Mathias',localIp = localIp)


if __name__ == '__main__':
    try:
        client = InfluxDBClient(host='127.0.0.1', port=8086,username='admin', password='admin',database ="communication")
        print(client)
        #query = "SELECT value FROM communication GROUP BY * order by DESC LIMIT 1"
        #print("{0}".(client.query(query)))
    except:
        print("Error while connecting to the influxDb")
    try:
        conn = mariadb.connect(
            user="root",
            password="admin",
            host="127.0.0.1",
            port=3306,
            database="test"
    )
    except mariadb.Error as e:
        print(f"Error connecting to MariaDB Platform: {e}")
    socketio.run(app, host='0.0.0.0', port=5000, use_reloader=False, debug=True)
    
    
    