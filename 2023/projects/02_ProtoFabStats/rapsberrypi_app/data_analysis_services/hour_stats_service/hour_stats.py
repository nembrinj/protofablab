from influxdb import InfluxDBClient
from dateutil.parser import parse
import datetime

# influx_db_client = InfluxDBClient(host="localhost", port=8086)
influx_db_client = InfluxDBClient(host="192.168.1.66", port=8086)


def InsertData(data):
  # Remove everything else
  influx_db_client.query('delete from "APDS9960_hour"')

  for entry in data:
    time = entry['time']
    average = entry['average']

    data = f'APDS9960_hour average={average} {time}'
    print(data)
    influx_db_client.write_points(data,
                                  database='profab',
                                  protocol='line')


def main():
  influx_db_client.switch_database('profab')

  # Get occupancy
  # 7 days, 24 values => first 7*24=168 values
  NUMBER_OF_DAYS_ANALYZED = 7
  query = f'SELECT MAX(occupancy::integer) AS max_occupancy FROM "APDS9960_processed" WHERE time > now()-7d GROUP BY time(1h) ORDER BY time LIMIT 168'
  occupancy_data = list(influx_db_client.query(query))[0]

  last_occupancy_per_hour_query = f'SELECT LAST(occupancy::integer) AS last_occupancy FROM "APDS9960_processed" WHERE time > now()-7d GROUP BY time(1h) ORDER BY time LIMIT 168'
  last_occupancy_per_hour_data = list(
    influx_db_client.query(last_occupancy_per_hour_query))[0]
  tmp = 0
  print("========== occupancy_data[:24]")
  print(occupancy_data[:24])
  print("========== last_occupancy_per_hour_data[:24]")
  print(last_occupancy_per_hour_data[:24])
  # Update occupancy_data based on last_occupancy_per_hour_data
  for index in range(1, len(occupancy_data)):
    # occupancy_data[index] might be 0 because there was no enter/leave event or there
    # was only leave event => making the max occupancy with 1 smaller than in reality
    if occupancy_data[index]["max_occupancy"] is None:
      # No event during the current hoour
      if last_occupancy_per_hour_data[index - 1]["last_occupancy"] is None:
        # No event during the previous time slot either, copy older value
        occupancy_data[index]["max_occupancy"] = tmp
      elif last_occupancy_per_hour_data[index - 1]["last_occupancy"] == 0:
        # There were events during the previous time slot, but at the end of it
        # ocucpancy was 0
        occupancy_data[index]["max_occupancy"] = 0
        tmp = 0
      else:
        # There was an event during the previous time slot and occupancy at the end was
        # not 0
        occupancy_data[index]["max_occupancy"] = last_occupancy_per_hour_data[index - 1]["last_occupancy"]
        # Previously there was an event, update tmp
        tmp = occupancy_data[index]["max_occupancy"]
    else:
      # There were events during the current hour
      if last_occupancy_per_hour_data[index - 1]["last_occupancy"]:
        # There were events during previous one too
        if last_occupancy_per_hour_data[index - 1]["last_occupancy"] > occupancy_data[index]["max_occupancy"]:
          # leave event ws the first evend during the current hour
          occupancy_data[index]["max_occupancy"] = last_occupancy_per_hour_data[index - 1]["last_occupancy"]
      else:
        # No events during the previous hour, need to see (by checking temp) if the
        # value from at least one hour before the current one
        occupancy_data[index]["max_occupancy"] = max(
          occupancy_data[index]["max_occupancy"], tmp)
      tmp = last_occupancy_per_hour_data[index]["last_occupancy"]

  print("========== occupancy_data[:24]")
  print(occupancy_data[:24])

  hour_average = [0 for _ in range(24)]
  for entry in occupancy_data:
    hour = parse(entry["time"]).hour
    max_occupancy = entry["max_occupancy"] if entry["max_occupancy"] else 0
    print(f'+ hour={hour} max_occupancy={max_occupancy}')
    hour_average[hour] += max_occupancy
  print(hour_average)

  processed_average = [{'average': entry / NUMBER_OF_DAYS_ANALYZED}
                       if entry > 0 else {'average': 0} for entry in hour_average]

  # Add time (yesterday's hours, it doesn't really matter as these values will be displayed
  # in Grafana based on their gour and not date)
  today = datetime.datetime.today()
  yesterday = (today - datetime.timedelta(days=today.weekday()))
  yesterday = datetime.datetime(yesterday.year, yesterday.month, yesterday.day)
  for hour_index in range(24):
    time_to_use = yesterday + datetime.timedelta(hours=hour_index)
    print(time_to_use)
    processed_average[hour_index]['time'] = int(
      time_to_use.timestamp() * 1000000000)

  print(processed_average)
  InsertData(processed_average)


if __name__ == "__main__":
  print("===== ProtoFabStats Hour Service Start =====")
  main()
  print("===== ProtoFabStats Hour Service End =====")
