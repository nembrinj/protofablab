
usr: pi
pwd: protolab

# Flask server app example

## install
set passwd and ssh at install time (rpi imager)
    
on laptop find your ip network

    ip addr

on laptop scan for available devices (change all ip addresses below according to local network)

    fping -gaq -r 0 192.168.1.0/24

on laptop connect to pi

    ssh pi@192.168.1.xxx

on pi 

    mkdir data_server

on laptop (in data_server folder)
    
    scp -r ./data_server.* ./templates  pi@192.168.1.xxx:/home/pi/data_server/


on pi install pandas + flask (this takes a while...)

    sudo apt install python3-pandas python3-flask


## make script run at boot

    
on pi

    sudo cp ./data_server.service /etc/systemd/system/data_server.service

    sudo systemctl daemon-reload
    sudo systemctl enable data_server.service

## check if running
    
    systemctl status data_server.service
    
    sudo systemctl list-unit-files | grep data_server
    sudo systemctl restart data_server.service
    
    
## get lux values

connect to

    http://192.168.1.xxx:5000
    http://192.168.1.xxx:5000/analysis
    
    




