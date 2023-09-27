from flask import Flask, render_template, Response
import pandas as pd
from datetime import datetime

app = Flask(__name__)

# same format as for data recording
dateparse = lambda x: datetime.strptime(x, "%Y_%m_%d-%H_%M_%S")

def analyze_lux_data() :
    """data analysis with pandas"""
    # load data using data parser lambda function
    df = pd.read_csv("../data_recorder/illuminance.csv", parse_dates=True, date_parser=dateparse, index_col='datetime')
    
    print(df.tail())
    
    
    # simple office hours time 
    # https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.between_time.html
    start = '8:00'
    stop = '18:00'
    
    # simple statistics
    _mean = round(float(df.between_time(start,stop).mean()),3)
    _std  = round(float(df.between_time(start,stop).std()),3)
    _min  = float(df.between_time(start,stop).min())
    _max  = float(df.between_time(start,stop).max())
    
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
