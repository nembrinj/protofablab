from influxdb import InfluxDBClient
import pandas as pd
import numpy as np
from dateutil.parser import parse
from datetime import datetime

# Configs for InfluxDB
# If running on PC
# HOST = "192.168.1.66"
# If running on Raspberry Pi
HOST = "localhost"
PORT = 8086

# Configs for data
# Estimated time, in ms, for an event (i.e. person entering/leaving)
MEDIAN_TIME = 500
TIME_BETWEEN_EVENTS = 1000
MINIMUM_TIME_EVENT = 1000

SELECT_MEDIAN_VL53L4CX_QUERY = f'SELECT MEDIAN("measurement") AS value, time FROM "VL53L4CX" GROUP BY TIME({MEDIAN_TIME}ms)'

influx_db_client = InfluxDBClient(host=HOST, port=PORT)


def GroupData(data):
  groups = []
  current_group = []
  for entry in data:
    if not len(current_group):
      current_group.append(entry)
      continue

    datetime = parse(entry["time"])
    delta = datetime - parse(current_group[-1]["time"])
    difference_in_milliseconds = delta.seconds * 1000 + delta.microseconds / 1000

    if (delta.days > 0 or difference_in_milliseconds > TIME_BETWEEN_EVENTS):
      # Too much time has passed, start a new group
      groups.append(current_group.copy())
      current_group.clear()
      current_group.append(entry)
    else:
      # Current entry should be added to the current open group.
      current_group.append(entry)

  if len(current_group):
    groups.append(current_group.copy())

  # Drop groups for which the duration of an event is < MINIMUM_TIME_EVENT
  filtered_groups = []
  for group in groups:
    delta = parse(group[-1]["time"]) - parse(group[0]["time"])
    difference_in_milliseconds = delta.seconds * 1000 + delta.microseconds / 1000
    if difference_in_milliseconds >= MINIMUM_TIME_EVENT:
      filtered_groups.append(group)

  return filtered_groups


# 1: goes up
# -1: goes down
# 0: unknown
def ComputeEvent(group):
  xvals = [entry["value"] for entry in group]
  yvals = [parse(entry["time"]).timestamp() * 1000 for entry in group]
  dx = np.diff(xvals)
  dy = np.diff(yvals)
  slopes = dy / dx
  print(slopes)

  slopes_sign = []
  for slope in slopes:
    if slope > 0:
      slopes_sign.append(1)
    elif slope < 0:
      slopes_sign.append(-1)
    else:
      slopes_sign.append(0)
  len_all_positive = len([1 for value in slopes_sign if value >= 0])
  len_all_negative = len([1 for value in slopes_sign if value <= 0])
  all_positive = (len_all_positive > len(slopes_sign) * 0.9)
  all_negative = (len_all_negative > len(slopes_sign) * 0.9)
  return {"start": group[0]["time"],
          "end": group[-1]["time"],
          "event": 1 if all_positive else -1 if all_negative else 0}


def ComputeEvents(groups):
  return [ComputeEvent(group) for group in groups]


def InsertEvents(events, measurement_name):
  for event in events:
    event_type = event["event"]
    time = event["start"]
    time = parse(time)
    time = int(datetime.timestamp(time) * 1000000000)

    data = f'{measurement_name} event_type={event_type} {time}'
    influx_db_client.write_points(data,
                                  database='profab',
                                  protocol='line')


def ComputeNumberOfUsers(events):
  number_of_users = []
  current_number_of_users = 0

  for event in events:
    if event["event"] == 1:
      current_number_of_users += 1
    elif event["event"] == -1:
      current_number_of_users -= 1
      current_number_of_users = max(current_number_of_users, 0)
    number_of_users.append({
      'number': current_number_of_users,
      'time': event['start']
    })

  return number_of_users


def InsertNumberOfUsers(number_of_users, measurement_name):
  for entry in number_of_users:
    number = entry["number"]
    time = entry["time"]
    time = parse(time)
    time = int(datetime.timestamp(time) * 1000000000)

    data = f'{measurement_name} number={number} {time}'
    influx_db_client.write_points(data,
                                  database='profab',
                                  protocol='line')


if __name__ == "__main__":
  influx_db_client.switch_database('profab')
  data = influx_db_client.query(SELECT_MEDIAN_VL53L4CX_QUERY)
  data = list(data.get_points())

  # Drop nulls
  data = [entry for entry in data if entry["value"] != None]
  groups = GroupData(data)

  # Compute events
  events = ComputeEvents(groups)
  # InsertEvents(events, measurement_name="user_events_1")

  # Compute number of users
  number_of_users = ComputeNumberOfUsers(events)
  InsertNumberOfUsers(number_of_users, measurement_name="number_of_users_1")
