# Containerization


## RasPi Zero W access point setup

Wifi access point configuration will be done thanks to the autowlan docker image ([repo](https://gitlab.com/hartek/autowlan/-/tree/master/))

First you need access to your pi zero with ssh

* temporarily input SSH and wifi settings in the Pi imager 
* flash your SD card with the lite 32-bit OS version 
* start your pi, let it configure for the first run, make sure it is visible on the wifi network
* connect to it on SSH


Then, follow these steps:

1. install docker and docker compose 
2. setup the autowlan container
3. make the autowlan docker image run at startup using systemd

You have to go through ALL these steps to get a hotspot running on the pi, otherwise you might lose connection to it, and will need to start again.

### configure raspberry OS to connect on USB

On your computer edit /boot/config.txt and add 

    dtoverlay=dwc2

at the end of the file. Then edit /boot/cmdline.txt add the following _after_ rootwait with a __single space__ before (and after if needed)

    modules-load=dwc2,g_ether

The above parameter should be added after the rootwait parameter. Yes the above parameter is a single parameter, meaning don’t add a bunch of space characters to it. More information on networking over USB on Linux can be found here.


### install docker and docker compose 

Following [this tutorial](https://www.jfrog.com/connect/post/install-docker-compose-on-raspberry-pi/)

    # Install some required packages first
    sudo apt update
    sudo apt upgrade
    sudo apt install git

    # Get Docker
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    
    # allow pi user to run docker commands
    sudo usermod -aG docker pi
    sudo su - ${USER}
        
    # start docker deamon at startup
    sudo systemctl enable --now docker
    # test docker install
    sudo docker run --rm hello-world
    
    # check versions
    sudo docker version
    sudo docker compose version

Docker and docker-compose are now installed on your raspberry Pi Zero W system

### setup the autowlan container

0. do a little setup following https://fwhibbit.es/en/automatic-access-point-with-docker-and-raspberry-pi-zero-w

    sudo nano /etc/dhcpcd.conf
    
add the line

    denyinterfaces wlan0

and in this config file

    sudo nano /etc/sysctl.conf
    
Uncomment the line to enable packet forwarding for IPv4

    net.ipv4.ip_forward=1

1. create the ~/compose/docker-compose.yml file

    version: '3.3'
    services:

      autowlan:
        image: jsdir/autowlan:latest
        network_mode: host
        restart: always
        volumes:
          - /boot/wpa2.conf:/etc/hostapd/hostapd.conf
          - /boot/dhcpd.conf:/etc/dhcp/dhcpd.conf
        cap_add:
          - NET_ADMIN

2. create a wpa2.conf file in your /boot partition to allow to change SSID/passwd on host computer

    sudo nano /boot/wpa2.conf

    interface=wlan0
    driver=nl80211
    ssid=protofabAP
    hw_mode=g
    ieee80211n=1
    channel=6
    auth_algs=1
    ignore_broadcast_ssid=0
    wpa=2
    country_code=CH
    macaddr_acl=0

    wpa_passphrase=something
    wpa_key_mgmt=WPA-PSK
    wpa_pairwise=CCMP
    rsn_pairwise=CCMP

3. create a dhcpd.conf file in your /boot partition to configure dhcp as your liking

    sudo nano /boot/dhcpd.conf

    authoritative; 
    subnet 11.0.0.0 netmask 255.255.255.0 {
        range 11.0.0.10 11.0.0.20; 
        option broadcast-address 11.0.0.255; 
        option routers 11.0.0.1; 
        default-lease-time 600; 
        max-lease-time 7200; 
        option domain-name "local"; 
        option domain-name-servers 8.8.8.8; 
    }


4. do NOT start the docker composer yet (you will lose SSH connection). Wait until completing the next step.

                
### make the autowlan docker image run at startup using systemd

1. create the systemd service unit file

    sudo nano /etc/systemd/system/docker-compose-app.service

2. edit unit file

[Unit]
Description=Docker compose autowlan service
Requires=docker.service
After=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/pi/compose/
ExecStart=/usr/local/bin/docker-compose up -d
ExecStop=/usr/local/bin/docker-compose down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target

3. enable the app at startup

    sudo systemctl enable docker-compose-app

4. now you can reboot the raspi

    sudo reboot


An OS image is provided [here](https://drive.switch.ch/index.php/s/TL1WZRiloSVWwLH) with all these steps already conducted. You only have to change the wifi settings in the /boot/wpa2.conf file to define your own wifi AP. This image is ready with docker and docker-compose installed


# sharing connection

WIN or OSX

https://solarianprogrammer.com/2018/12/07/raspberry-pi-zero-internet-usb/

Ubuntu

using network manager https://www.tecmint.com/share-internet-in-linux/

"share with other computers" USB ethernet connection, disable/enable network interface

scan ip network using fping -gaq xxx.xxx.xxx.xxx





- Insert the Raspi sd card in your PC
- Run `diskutil list`: from the obtained list, note the name of the sd card (it should be something like `/dev/disk4`
- Copy and compress the content of the raw storage space to a `.gz`file. Substitute `/dev/rdisk4` with the name obtained in the previous point (with an `r` prefix, so `/dev/dsk4`=> `/dev/rdisk4`):
```
sudo dd if=/dev/rdisk4 bs=1m | gzip > /Users/Yourname/Desktop/pi_autowlan.gz
```
- The same but without compression: 
```
sudo dd if=/dev/rdisk4 of=/Users/Yourname/Desktop/pi_autowlan.img bs=1m
```
- Wait for the copy to be finished (it takes quite a long time)


 





