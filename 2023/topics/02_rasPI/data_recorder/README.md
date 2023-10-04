


# Periodic data recorder with python and systemctl

## install

On rpi imager, configure

* ProFab wifi
* local hostname, 
* set passwd and ssh 

at install time.

Power up your pi (this takes a long time....) 
 

## log in     
        

2 options:

1. use the "yourname".local address
2. find your RasPi ip  (see below)

ssh on your raspi

    ssh pi@yourname.local
    # or
    ssh pi@192.168.1.xxx
    
...done


find your own ip

    ip addr
    
find all devices on your network (change ip as needed)

    fping -gaq -r 0 192.168.1.0/24
    
192.168.1.1 is the access point.


## copy script
    

on pi 

    mkdir data_recorder

on laptop (in data_recorder folder)
    
    scp ./data_recorder.py ./data_recorder.service ./data_recorder.timer  pi@yourname.local:/home/pi/data_recorder/
    # or
    scp ./data_recorder.py ./data_recorder.service ./data_recorder.timer  pi@192.168.1.xxx:/home/pi/data_recorder/
    
on pi, modify ip address of ESP32 in data_recorder.py

    cd data_recorder
    nano ./data_recorder.py
    


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

get all values on laptop

    scp pi@yourname.local:/home/pi/data_recorder/illuminance.csv .
    # or
    scp pi@192.168.1.xxx:/home/pi/data_recorder/illuminance.csv .

