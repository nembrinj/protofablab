from flask import Flask, render_template, Response
from datetime import datetime
import csv

app = Flask(__name__)

def analyze_lux_data() :
    """simple data analysis without pandas"""
    
    _min = 1000000.0
    _max = -1.0
    _sum = 0.0
    _length = 0
    with open('data/illuminance.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[1]=='lux':
                continue
            val = float(row[1])
            if val > _max:
                _max = val
            if val < _min:
                _min = val
            _sum = _sum + val
            _length += 1 
    
    _mean = _sum/_length
    _std = -1 # not computed
    
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

if __name__ == '__main__':
    #app.run(debug=True)
    app.run(host="0.0.0.0")
