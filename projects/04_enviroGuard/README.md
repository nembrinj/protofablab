# A raspberry pi work environment monitor

EnvirogGuard monitors, via three sensors, your work environment and warns you via a web app when an ambient condition, such as light level, air quality or temperature, moves beyond acceptable thresholds.

The objective is to allow the user to have a clearer idea of their work environment's conditions. Another main aim is to allow the user to keep a real-time control over their environment, thanks to small suggestions (opening a window, lowering heat, taking a small break while air refreshes, etc) included in the alerts sent by the web application. This is all accomplished by simply connecting to a web page and putting it into the background while continuing on doing other work.

## Setup

This project combines three sensors: a Bosch BME280 (pressure, temperature and humidity) packaged on a board by adafruit, a Vishay VEML700 lux sensor, and a CCS811 Air quality sensor (CO2 and TVOC) by stemmaQT. All sensors are connected to the raspberry pi via four pins, two for power (5V and ground) and two for use with the I2C protocol. We recommend the use of four 1-to-3 wire connectors in order to simplify the necessary wire connections.

Once the sensors have been correctly wired to the raspberry pi, it is time to install the software. Since the project depends on the use of grafana, influxdb and mariaDB, we recommend the use of an easier to use linux distribution like DietPi [[link]], which we will use for the rest of this project.

Once dietpi has been loaded onto an SD card, several files have to be modified before booting the raspberry pi. You should add your home/work wifi configuration in the wifi.txt file on the boot partition, or you will not be able to access the raspberry pi.

Once you are connected to the raspberry pi, proceed with the guided installation. Once setup is done, enable I2C using the `dietpi-config` command, under advanced options. Make sure to set the I2C baudrate to 50k, as one sensor we use has trouble with higher values. You can then install the required software using the `dietpi-software` command-line tool. A list can be found in the image below.

-- insert image required_software.png -- 

You are then clear to install the project, by simply copying the `src` folder onto your raspberry pi. Finally, some addtional python libraries are required, which you can easily install using the following command from inside the src folder.
```
pip3 install -r requirements.txt
