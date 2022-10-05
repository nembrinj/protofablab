
import requests
import time
import csv
from pathlib import Path


url = "http://192.168.1.yyy/readlux"

r = requests.get(url)

print(r.text)
lux = r.text

timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
print(timestr)

columns = ["datetime", "lux"]

FILE_PATH = Path('illuminance.csv')

if not FILE_PATH.exists():
    with open(FILE_PATH, 'w', newline='') as csv_file:
        csv_file_writer = csv.writer(csv_file)
        csv_file_writer.writerow(columns)

with open(FILE_PATH, 'a', newline='') as csv_file:
    csv_file_append = csv.writer(csv_file)
    csv_file_append.writerow([timestr,lux])




