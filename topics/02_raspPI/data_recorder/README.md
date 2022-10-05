

usr: pi
pwd: protolab

# Periodic data recorder with python and systemctl

## install
set passwd and ssh at install time (rpi imager)
    
on laptop find your ip network

    ip addr

on laptop scan for available devices

    fping -gaq -r 0 192.168.1.0/24

on laptop connect to pi

    ssh pi@192.168.1.xxx    

on pi 

    mkdir data_recorder

on laptop (in data_recorder folder)
    
    scp ./data_recorder.py ./data_recorder.service ./data_recorder.timer  pi@192.168.1.xxx:/home/pi/data_recorder/


## make script run at definite times

use this SO answer https://unix.stackexchange.com/questions/198444/run-script-every-30-min-with-systemd
    
on pi

    sudo cp ./data_recorder.service /etc/systemd/system/data_recorder.service
    sudo cp ./data_recorder.timer /etc/systemd/system/data_recorder.timer

    sudo systemctl daemon-reload
    sudo systemctl enable data_recorder.timer

## check if running
    
    systemctl status data_recorder.timer
    systemctl status data_recorder.service
    
    systemctl list-timers --all

    sudo systemctl restart data_recorder.timer
   
    
## get lux values

get all images on laptop

    scp pi@192.168.1.xxx:/home/pi/data_recorder/illuminance.csv .

