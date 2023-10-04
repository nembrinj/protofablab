

# Flask server app example

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
    ssh pi@192.168.1.xxx
    
...done


find your own ip

    ip addr
    
find all devices on your network (change ip as needed)

    fping -gaq -r 0 192.168.1.0/24
    
192.168.1.1 is the access point.

## copy server code

on pi 

    mkdir data_server

on laptop (in data_server folder)
    
    scp -r ./data_server.* ./templates  pi@yourname.local:/home/pi/data_server/
    # or
    scp -r ./data_server.* ./templates  pi@192.168.1.xxx:/home/pi/data_server/


on pi install pandas + flask (this takes a while...)

    sudo apt install python3-pandas python3-flask
    
see 

https://forums.raspberrypi.com/viewtopic.php?t=320294#p1918001 

if you have problem downloading the packages


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

    http://yourname.local:5000
    http://yourname.local:5000/analysis
    # or
    http://192.168.1.xxx:5000
    http://192.168.1.xxx:5000/analysis
    
    




