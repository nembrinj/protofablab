from flask import Flask, render_template, Response, make_response
from datetime import datetime
import os
import time
import csv
import math

app = Flask(__name__)

def analyze_lux_data() :
    """simple data analysis without pandas"""
    
    _min = 1000000.0
    _max = -1.0
    with open('data/illuminance.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        tmp = []
        for row in csv_reader:
            if row[1]=='lux':
                continue
            tmp.append(float(row[1]))
    
    _mean = sum(tmp) / len(tmp)   # mean
    var  = sum(pow(x-_mean,2) for x in tmp) / len(tmp)  # variance
    _std  = math.sqrt(var)  # standard deviation
    _max = max(tmp)
    _min = min(tmp)
    
    return _mean,_std,_min,_max
   
@app.route('/analysis')
def analysis_page():
    """Analysis page."""
    m,s,mi,ma = analyze_lux_data()
    return render_template('analysis.html', values = [m,s,mi,ma])

@app.route('/')
def index():
    """current lux reading page"""
    return render_template('index.html')

@app.route('/current')
def current():
    current = 'none'
    with open('data/current') as f:
        current = f.readline()
    response = make_response(current, 200)
    response.mimetype = "text/plain"
    return response

@app.route('/lastreading')
def lastreading():
    return time.ctime(os.path.getmtime('data/illuminance.csv'))


if __name__ == '__main__':
    #app.run(debug=True)
    app.run(host="0.0.0.0")
