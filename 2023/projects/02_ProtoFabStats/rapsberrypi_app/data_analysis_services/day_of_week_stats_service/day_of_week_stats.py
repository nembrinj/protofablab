from influxdb import InfluxDBClient
from dateutil.parser import parse
import datetime

# influx_db_client = InfluxDBClient(host="localhost", port=8086)
influx_db_client = InfluxDBClient(host="192.168.1.66", port=8086)


def InsertData(data):
  # Remove everything else
  influx_db_client.query('delete from "APDS9960_days_of_week"')

  for entry in data:
    time = entry['time']
    average = entry['average']

    data = f'APDS9960_days_of_week average={average} {time}'
    print(data)
    influx_db_client.write_points(data,
                                  database='profab',
                                  protocol='line')


def main():
  influx_db_client.switch_database('profab')

  # Get occupancy
  query = f'SELECT MAX(occupancy::integer) AS max_occupancy FROM "APDS9960_processed" GROUP BY time(1d)'
  occupancy_data = list(influx_db_client.query(query))[0]

  day_of_week_average = [{'total': 0, 'count': 0} for _ in range(7)]
  for entry in occupancy_data:
    weekday_index = parse(entry["time"]).isoweekday()
    max_occupancy = entry["max_occupancy"] if entry["max_occupancy"] else 0

    day_of_week_average[weekday_index - 1]['total'] += max_occupancy
    day_of_week_average[weekday_index - 1]['count'] += 1

  processed_average = [{'average': entry['total'] / entry['count']}
                       if entry['count'] > 0 else {'average': 0} for entry in day_of_week_average]

  # Add time (last full week, it doesn't really matter as these values will be displayed
  # in Grafana based on their weekday and not date)
  today = datetime.datetime.today()
  last_monday = (today - datetime.timedelta(days=today.weekday() + 7))
  for day_index in range(7):
    current_day_datetime = last_monday + datetime.timedelta(days=day_index)
    processed_average[day_index]['time'] = int(
      current_day_datetime.timestamp() * 1000000000)

  InsertData(processed_average)


if __name__ == "__main__":
  print("===== ProtoFabStats DayOfWeek Service Start =====")
  main()
  print("===== ProtoFabStats DayOfWeek Service End =====")
