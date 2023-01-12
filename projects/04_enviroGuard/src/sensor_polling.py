from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import mariadb
from influxdb import InfluxDBClient
import time
import json
import sensors
import _thread
from datetime import datetime

try:
    client = InfluxDBClient(host='127.0.0.1', port=8086,username='admin', password='admin')
    print(client)
    client.switch_database("communication")
except:
    print("Error while connecting to the influxDb")

def handleSensors():

    while True:

        temperature, humidity, pressure, lux, tvoc, co2 = sensors.getData()
        
        data_points = [
            {
                "measurement": "temperature",
                "fields": {
                    "value": temperature
                }
            },
            {
                "measurement": "humidity",
                "fields": {
                    "value": humidity
                }
            },
            {
                "measurement": "pressure",
                "fields": {
                    "value": pressure
                }
            },
            {
                "measurement": "lux",
                "fields": {
                    "value": lux
                }
            },
            {
                "measurement": "tvoc",
                "fields": {
                    "value": tvoc
                }
            },
            {
                "measurement": "co2",
                "fields": {
                    "value": co2
                }
            }
        ]

        print(data_points)

        client.write_points(data_points)
        print(data_points)
        time.sleep(30) #change to 30 for final version

if __name__ == '__main__':
    handleSensors()
