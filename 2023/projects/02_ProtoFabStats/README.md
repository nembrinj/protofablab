# ProtoFabStats
*Author: Adriana Moisil*

## Table of Contents
- [ProtoFabStats](#protofabstats)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
    - [Background](#background)
    - [Motivation](#motivation)
    - [High Level Overview](#high-level-overview)
  - [Project Directory Structure](#project-directory-structure)
  - [Implementation Details](#implementation-details)
    - [Hardware](#hardware)
    - [Methodology](#methodology)
      - [MQTT](#mqtt)
      - [ESP32](#esp32)
      - [Raspberry Pi](#raspberry-pi)
  - [Appendix](#appendix)

## Introduction

### Background

...

### Motivation

While the other 2 projects (*UniLock* and *DEUS-EX*) focus on access at a software level, respectively phisically locking/unlkocking the door, this project tackels a differet aspect of the main problem. We chose to observe how the space is used and compute statistics that could be helpful for decision making in the future (i.e. answering questions like "Is today a good day to use the laboratory, or is it going to be too crowded?").

### High Level Overview 

...

## Project Directory Structure

The main directory contains 3 pdfs, one for each of the three different presentations that took place during the semester (*idea proposal*, *midterm presentation* and *final presentation*), and two subdirectories: *arduino_app*, containing the 3 files needed for the app running on the Adafruit ESP32 S3 Feather, and *raspberrypi_app*. The set up for the part that runs on Arduino is straightforward. However, the same cannot be said for the second part of the project. Thus, the *raspberrypi_app* subdirectory containing an additional README.md file explaining how to set up the pi for this project.

## Implementation Details

### Hardware

For this project the following pieces of hardware are required (in addition a breadboard and cables to connect the components together):
* *Raspberry Pi Zero* 
  <div style="text-align:left">
    <img src="images/raspberry_pi_zero.jpg?" height="200px;"/>
  </div>
  
* *Adafruit ESP32-S3 Feather* 
  <div style="text-align:left">
    <img src="images/esp32.jpg" height="200px;"/>
  </div>

* *Adafruit VL53L4CX*
  <div style="text-align:left">
    <img src="images/VL53L4CX.jpg" height="200px;"/>
  </div>
  This is a Time of Flight sensor that can detect objects that are up to 6m away from it. 
  Furthermore, it offers support for multi-object detection. This is not something that the ProtoFabStats project needs, but there is a simple solution: we are keeping only the closest object detected for each reading. The ... is that the sensor

* *Adafruit APDS9960*
  <div style="text-align:left">
    <img src="images/APDS9960.jpg" height="200px;"/>
  </div>
  This is a sensor for Proximity, Light, RGB, and Gesture. However, we only used its gesture capabilities. The sensor is capable of detecting gestures in the following directions: up, down, left and right. 


There is one more sensor that we have tried to use but fail: *Adafruit VL53L0X*. Another Time of Flight sensor, a predecesor of *Adafruit VL53L4X*, this sensor does not offer multi-object detection and proved to be difficult to work with.


### Methodology

#### MQTT

With the help of MQTT we can easily make the data collected by the Adafruit Sensor available on the server running on the Raspberry Pi.

The project uses the following 4 topics:
* esp32/APDS9960
  * for recorded measurements by the Adafruit APDS9960 sensor
  * published only by the Arduino application
  * this project is responsible for publishing all the information for this topic (and, at the same time, one of its components subscribes to this topic)
* esp32/VL53L4CX 
  * for recorded measurements by the Adafruit VL53L4CX sensor
  * published only by the Arduino application
  * this project is responsible for publishing all the information for this topic (and, at the same time, one of its components subscribes to this topic)
* door/event/locked_unlocked 
  * for passing around information about whether the door is locker or unlocked 
  * this project publish only partial information for this topic (whether the door is open/closed, and not whether it is locker/unlocked), while it subscribes to everything
  * example message "locked_unlocked status=1"; 1 for locked, 2 for unlocked, 0 for unknown
* door/event/open_closed 
  * for passing around information about whether the door is locker or unlocked 
  * this project is responsible for publishing all the information for this topic (and, at the same time, one of its components subscribes to this topic)
    * example message "open_closed status=1"; 1 for open, 2 for closed, 0 for unknown

All messages follow the InfluxDB line protocol.

#### ESP32

The logic that runs on the ESP32 plays a major role for this project. Its  responsability is to collect the data from the two sensors and publish it using their corresponding topics. The data is published almost as it is, with little to no decisions/alterations being made by this program. For the ToF sensor, we send the distance, while for the Gesture sensor we send the event type, which is either entering or leaving the room. The event type corresponds to either the up-down pair, or left-right pair, depending on the orientation of the sensor.

Furthermore, to make the program slightly smarter, it subscribes to 2 other MQTT topics: *door/event/open_closed* and *door/event/locked_unlocked*.
The assumption is that data is published by the other projects, and we are just using it it. The idea is that, if the status of the door is known, we can read data from sensors less often if we have an indication that the door is not open:
  * every 50ms when the door is open
  * every 0.1 seconds when the door is unlocked but closed
  * every second when the door is locked


#### Raspberry Pi

We use this board to subscribe to the MQTT topics where sensor readings are published, interpret and preprocess it, and lastly display it.

4 different services run on the Raspberry Pi:
* *Data Preprocessing Service*
  * Runs every 10 minutes
  * Computes occupancy metric based on enter/leave events
  * Despite its name, it is also used to validate the data collected by the sensors. The validations are executed based on certain assumptions:
    * The laboratory is accessed only when the building is open (e.g. 7:00 - 22:00), which means that, for example, nobody is there at midnight. If, at the end of the day, the data indicates that the room is not empty, we will believe that one or more of the sensors' readings were faulty. What this service does in that case is to reset the current occupancy number to 0 (by adding fake entry/leave events). 
  * Updates InfluxDB measurement APDS9960_processed
* *Days of Week Stats Service*
  * Runs daily at 4:00 (for testing, we made it run every 5 minutes)
  * Computes average occupancy metric per weekday
    * When computing new data, it removes the old one, so there is never more than one value/weekday (we made it easy for Grafana to display these statistics)
  * Updates InfluxDB measurement APDS9960_days_of_week
* *Hour Stats Service*
  * Runs every hour (for testing, we made it run every 5 minutes)
  * Computes average occupancy metric per hour based on the last 7 days
  * Updates InfluxDB measurement APDS9960_hour

* *Grafana* (see the README.md section under the *main_server* subdirectory for understanding why this decition had to be made)

In addition, 4 other application run in docker containers:
* *Mosquitto*
* *Telegraf*
* *InfluxDB V1*
* *Flask Service*

## Appendix

There are a few tips&tricks learnt during the semester that might come in handy for future projects:
* When setting up Raspberry Pi, it might end up with different suffixes in the name: either *.home* or *.local*. 
* Search for the Raspberry Pi on the local network using `fping -gaqne -r 0 192.168.1.0/24`. Adding the additional flag *n* will output the name of the target instead of its ip (easier to check if the pi is connected to the network or not). 
* The Raspberry Pi can connect to different networks, without having to reset it every time. In order to do so, the following configuration file can be modified (followed by restarting the pi): */etc/wpa_supplicant/wpa_supplicant.conf*.
* While working with the sensors for the arduino board, sometimes they had to be disconnected and reconnected again (the sensors to the arduino, not necessarily the arduino to the PC) in order for a restart/installation of a program to properly reinstantiate their start state.