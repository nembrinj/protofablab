## Table of Contents
- [Tutorial for the HomeNotifier alongside the water sensor.](#tutorial-for-the-homenotifier-alongside-the-water-sensor)
  - [Introduction](#introduction)
    - [**Why** did we choose it?](#why-did-we-choose-it)
  - [Technologies](#technologies)
  - [General schema](#general-schema)
  - [Electrical schema](#electrical-schema)
  - [Procedure](#procedure)
  - [Preview of the code and functionality](#preview-of-the-code-and-functionality)
  - [**Demo** video](#demo-video)

# Tutorial for the HomeNotifier alongside the water sensor.

WhatsApp notifier created for ESP32 Feather using Arduino. This is the final project of the course Fabrication and Prototyping in the LearningLab at the University of Bern in the fall semester of 2022.



## Introduction
After a lot of brainstorming sessions in which we proposed, we came up with several projects and a few more discussions on the choice of project, this is the result. It took multiple coffee ☕, chocolate 🍫 and Friday (sometimes even Thursday) 🕑 breaks offered to the project to bring it to fruition.

Link to the code can be found [here](https://github.com/nembrinj/protofablab/blob/main/topics/01_arduino/Final%20Project/Final%20Project.ino).
### **Why** did we choose it?
We've been thinking about the different scenarios we're put in on a daily basis and have found out what we think can be a challenge ⚠️ for many of us. 

The basic idea is to eliminate the time it takes to constantly check the water level of a bathtub 🛀.

Suppose the following hypothetical situation: John 🧑 wants to take a bath in a bathtub, he is forced and forced to constantly check the water level so that it does not back 💦 up out of the bathtub.
This problem is completely eliminated if we use the water sensor together with HomeNotifier.

* * *

## Technologies




## General schema

The scheme of the project is modular. Each module consists of a sensor that is connected to an ESP32 Feather. Depending on the sensor and its use, the ESP32 Feather will be programmed to send and receive data via Wi-Fi to the Twilio server to later be transmitted as WhatsApp messages to the user.

//Figure slide 3

In this tutorial, we focus on the application of detecting the water level in a bathtub as it fills up. Other possible uses may be to detect the temperature of a pot until it boils.


//Figure slide 2

![Figure2](https://www.dropbox.com/s/qdvqh2kiobunt8s/Device_schema.PNG?dl=0)

To determine the water levels in the bathtub application we have used the VL53L0X TOF sensor. This is an infrared emitter and receiver that measures the flight time that elapses between a pulse leaving the sensor and returning reflected. The sensor uses this time to calculate the distance at which the reflective surface is.

## Electrical schema

The connection between the TOF VL53L0X sensor and the ESP32 Feather is straightforward:

GND in sensor to GND in ESP32 Feather
Vin in sensor to GND in ESP32 Feather
SCL in sensor to SCL in ESP32 Feather
SDA in sensor to SDA in ESP32 Feather

//Figure Electrical Schema
(Source: https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/python-circuitpython)

## Procedure

The process that que are going to implement consists of 5 steps:
 
1. Set up the Wifi and phone number
2. Receive WhatsApp confirmation message from Twilio
3. Set up the water levels
4. Reset the device
5. Ready to use


* * *
## Preview of the code and functionality


- How to Set up the Wifi credentials using a smartphone?
  
This can be done using the [WiFiManager](https://github.com/tzapu/WiFiManager) library. For example:
```
  WiFiManager wfm;

  wfm.setDebugOutput(false);
  wfm.resetSettings();
  
  WiFiManagerParameter my_phone_number("my_number", "Enter your phone number!", "default" , 15);
  wfm.addParameter(&my_phone_number);
```
We first instantiate an object called **wfm**

The **my_phone_number()** constructor will create an object that has the following parameters: "my_number", placeholder text, default value and a max length. 

```
  if(!wfm.autoConnect("HN Water Sensor", "password")){
    ESP.restart();
    delay(1000);
  }
  myNumber = my_phone_number.getValue();
```
Where **myNumber** will store the value of the **my_phone_number** input as a string.
If the password is wrong the WiFi login interface will be restarted by resseting the ESP. 
  
- How to send and recevie messages from the ESP32 Feather to WhatsApp?


- How to save water level values into the ESP32 Feather flash memory?



* * *
## **Demo** video
This is a demonstration that our user notification system is working properly! For the experiment we used:

- 2 water buckets: (one empty and one full). The empty bucket is used at the beginning to simulate an empty bathtub.
- Lamp holder: It is used to hold the sensor up and aimed at the bottom of the bucket.

* * *
 
https://user-images.githubusercontent.com/15964709/210858171-c8710c31-4560-43ef-b8ef-35b9d91e1f13.mp4



















Missing from the previous tutorial

❌ Sandro: General schema + Process

❌ Alex: Implementation 


* a schema of the concept of your project is missing in the tutorial. As is missing an electrical schema….Linking your presentation is not a solution. Take a look at your tutorial: would you be willing to read it without schema/electrical schema?
  ❌Sandro

* You should generalize your idea: why is this project interesting for programming arduino boards in general? Your intro should be much more interesting to invite the reader to engage in following the tutorial.  ✅ALEXANDRU 

* The tutorial follows exactly the code. This is probably not the best option in terms of explaining your project (and even explaining the code).  ❌ALEXANDRU 

* There is no presentation of a successful run (video?image?)       ✅ALEXANDRU 
