
import requests
import time, threading
import csv
from pathlib import Path

url = "http://192.168.1.4/readlux"  # <----------- CHANGE HERE
WAIT_SECONDS = 10

def get_reading():

    r = requests.get(url)

    print(r.text)
    lux = r.text

    timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
    print(timestr)

    columns = ["datetime", "lux"]

    FILE_PATH = Path('data/illuminance.csv')

    if not FILE_PATH.exists():
        with open(FILE_PATH, 'w', newline='') as csv_file:
            csv_file_writer = csv.writer(csv_file)
            csv_file_writer.writerow(columns)

    with open(FILE_PATH, 'a', newline='') as csv_file:
        csv_file_append = csv.writer(csv_file)
        csv_file_append.writerow([timestr,lux])

    threading.Timer(WAIT_SECONDS, get_reading).start()
    
get_reading()


