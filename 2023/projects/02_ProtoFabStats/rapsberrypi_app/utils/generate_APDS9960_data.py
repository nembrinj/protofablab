from influxdb import InfluxDBClient
import pandas as pd
import numpy as np
from dateutil.parser import parse
from datetime import datetime, timedelta
import random

HOST = "192.168.1.66"
PORT = 8086

# For how many days should the data be generated
NUMBER_OF_DAYS = 21

# APDS9960
APDS9960_PROCESSED_OCCUPANCY_ENABLED = True
APDS9960_MIN_VALS_PER_DAY = 1
APDS9960_MAX_VALS_PER_DAY = 30
PERCENTAGE_OF_DAYS_WITH_VALUES = 90
PERCENTAGE_OF_OK_DAYS = 60  # How many times the number of people left in the room is 0

influx_db_client = InfluxDBClient(host=HOST, port=PORT)


################################################################################


# Forced==True means that these events are added to reset the occupancy to 0 at
# midnight.
def InsertAPDS9960Data(events, times, forced=False):
  occupancies = [0] * len(events)
  for index in range(len(events)):
    if index == 0:
      occupancies[index] = max(0, events[index])
    else:
      occupancies[index] = max(0, occupancies[index - 1] + events[index])

  for index in range(len(events)):
    time = times[index]

    event = events[index]
    data = f"APDS9960 event={event},forced=true {time}" \
        if forced \
        else f"APDS9960 event={event} {time}"
    influx_db_client.write_points(data, database="profab", protocol="line")

    if APDS9960_PROCESSED_OCCUPANCY_ENABLED:
      occupancy = occupancies[index]
      data = f"APDS9960_processed occupancy={occupancy},forced=true {time}" \
          if forced \
          else f"APDS9960_processed occupancy={occupancy} {time}"
      influx_db_client.write_points(
          data, database="profab", protocol="line")


def GenerateAPDS9960Data():
  now = datetime.now()
  today = datetime(now.year, now.month, now.day)
  pass
  for day_index in range(NUMBER_OF_DAYS):
    number_of_values = random.randint(
        APDS9960_MIN_VALS_PER_DAY, APDS9960_MAX_VALS_PER_DAY
    )
    day_has_values = random.randint(1, 100) <= PERCENTAGE_OF_DAYS_WITH_VALUES
    if not day_has_values:
      continue

    is_an_ok_day = random.randint(1, 100) <= PERCENTAGE_OF_OK_DAYS
    if is_an_ok_day:
      # Number of enter=1 == nubmer of leave=-1
      # Occupancy will be 0
      if number_of_values % 2 != 0:
        number_of_values += 1

      sum = 0
      events = [0] * number_of_values
      for index in range(number_of_values):
        if sum == 0:
          events[index] = 1
          sum += 1
          continue
        if number_of_values - index == sum:
          events[index] = -1
          sum -= 1
          continue
        event = [-1, 1][random.randint(0, 1)]
        events[index] = event
        sum += event
    else:
      # 1 = enter, -1 = leave
      events = [[-1, 1][random.randint(0, 1)]
                for _ in range(number_of_values)]

    # Get [start, end] interval for times
    target_day = today - timedelta(days=day_index + 1)
    target_day_start = datetime(
        target_day.year, target_day.month, target_day.day, 7
    )
    target_day_end = datetime(
        target_day.year, target_day.month, target_day.day, 22)
    milliseconds_start = int(target_day_start.timestamp() * 1000000000)
    milliseconds_end = int(target_day_end.timestamp() * 1000000000)

    # Generate times
    times = sorted(
        [
            random.randint(milliseconds_start, milliseconds_end)
            for _ in range(number_of_values)
        ]
    )

    # Add to DB
    InsertAPDS9960Data(events, times)

    # See if extra events should be added at midnight to get the occupancy back to 0.
    last_occupancy = max(0, events[0])
    for index in range(1, len(events)):
      last_occupancy = max(0, last_occupancy + events[index])
    sum_events = last_occupancy

    forced_events = []
    forced_times = []
    if not is_an_ok_day and sum_events != 0:
      event_to_add = 1 if sum_events < 0 else -1
      target_day_midnight = datetime(
          target_day.year, target_day.month, target_day.day, 23, 59, 59
      )
      time = int(target_day_midnight.timestamp() * 1000000000)
      for _ in range(abs(sum_events)):
        forced_events.append(event_to_add)
        forced_times.append(time)

    # Add forced events to DB
    InsertAPDS9960Data(forced_events, forced_times, forced=True)


################################################################################


if __name__ == "__main__":
  influx_db_client.switch_database("profab")
  GenerateAPDS9960Data()
