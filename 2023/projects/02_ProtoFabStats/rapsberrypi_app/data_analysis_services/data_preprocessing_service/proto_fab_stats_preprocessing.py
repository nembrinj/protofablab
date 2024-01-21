from influxdb import InfluxDBClient
import pandas as pd
import numpy as np
from dateutil.parser import parse
from datetime import datetime, timedelta
import random

# influx_db_client = InfluxDBClient(host="localhost", port=8086)
influx_db_client = InfluxDBClient(host="192.168.1.3", port=8086)


################################################################################


def InsertData(data):
  for entry in data:
    time = entry['time']
    occupancy = entry['occupancy']

    time_obj = parse(time)
    timestamp = int(time_obj.timestamp() * 1000000000)

    data = f'APDS9960_processed occupancy={occupancy} {timestamp}'
    print(data)
    influx_db_client.write_points(data,
                                  database='profab',
                                  protocol='line')


def Main():
  influx_db_client.switch_database('profab')

  # Check the last preprocessed data
  last_preprocessed_data = list(influx_db_client.query(
    'SELECT * from "APDS9960_processed" ORDER BY "time" DESC LIMIT 1'))
  # Skip if nothing
  if not len(last_preprocessed_data) or not len(last_preprocessed_data[0]):
    return

  print(last_preprocessed_data)
  last_time = last_preprocessed_data[0][0]["time"]
  last_occupancy = int(last_preprocessed_data[0][0]["occupancy"])
  print(
    f'ProtoFabStats Service:: last_time = {last_time}, last_occupancy={last_occupancy}')

  # Get all new events
  query = f'SELECT "event" FROM "APDS9960" WHERE time > (\'{last_time}\' + 1u)'
  events = list(influx_db_client.query(query))
  print(f'ProtoFabStats Service:: len(events) = {len(events)}')
  if not len(events) or not len(events[0]):
    # Not interesting
    return
  events = events[0]

  # Preprocess events
  occupancies = []
  for index in range(len(events)):
    event = events[index]
    if index == 0:
      occupancy = {"time": event["time"], "occupancy": max(0, last_occupancy + event["event"])}
      occupancies.append(occupancy)
      continue
    
    occupancy = {"time": event["time"], "occupancy": max(0, occupancies[-1]["occupancy"] + event["event"])}
    occupancies.append(occupancy)

  print(occupancies)
  InsertData(occupancies)


if __name__ == "__main__":
  print("===== ProtoFabStats Service Start =====")
  Main()
  print("===== ProtoFabStats Service End =====")
