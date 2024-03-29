## Table of Contents
- [Tutorial for the HomeNotifier alongside the water sensor.](#tutorial-for-the-homenotifier-alongside-the-water-sensor)
  - [Introduction](#introduction)
    - [**Why** did we choose it?](#why-did-we-choose-it)
  - [Technologies](#technologies)
  - [General schema](#general-schema)
  - [Electrical schema](#electrical-schema)
  - [Procedure](#procedure)
  - [Preview of the code and functionality](#preview-of-the-code-and-functionality)
  - [The technical difficulties and how they were overcome](#the-technical-difficulties-and-how-they-were-overcome)
  - [Project value](#project-value)
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
 The project is created with:
1. Arduino IDE 2.0.3
2. Twilio
3. ThingEsp.h   1.3.0
4. Preferences.h preinstalled
5. WiFiManager. h 2.0.9-beta
6. VL53L0X.h 1.3.1

* * *

## General schema

The scheme of the project is modular. Each module consists of a sensor that is connected to an ESP32 Feather. Depending on the sensor and its use, the ESP32 Feather will be programmed to send and receive data via Wi-Fi to the Twilio server to later be transmitted as WhatsApp messages to the user.

![Figure1](https://raw.githubusercontent.com/nembrinj/protofablab/main/projects/01_HomeNotifier/Images/General_schema.png)

In this tutorial, we focus on the application of detecting the water level in a bathtub as it fills up. Other possible uses may be to detect the temperature of a pot until it boils.

![Figure2](https://raw.githubusercontent.com/nembrinj/protofablab/main/projects/01_HomeNotifier/Images/Device_schema.png)

To determine the water levels in the bathtub application we have used the VL53L0X TOF sensor. This is an infrared emitter and receiver that measures the flight time that elapses between a pulse leaving the sensor and returning reflected. The sensor uses this time to calculate the distance at which the reflective surface is.

* * *

## Electrical schema

The connection between the TOF VL53L0X sensor and the ESP32 Feather is straightforward:

- GND in sensor to GND in ESP32 Feather
- Vin in sensor to GND in ESP32 Feather
- SCL in sensor to SCL in ESP32 Feather
- SDA in sensor to SDA in ESP32 Feather

![Figure3](https://raw.githubusercontent.com/nembrinj/protofablab/main/projects/01_HomeNotifier/Images/VL53L0X_schema.png)


* * *

## Procedure

The process that que are going to implement consists of 5 steps:
 
1. Set up the Wifi and phone number
2. Receive WhatsApp confirmation message from Twilio
3. Set up the water levels
4. Reset the device
5. Ready to use

* * *

## Preview of the code and functionality


- ### How to Set up the Wifi credentials using a smartphone?
  
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
The **autoConnect()** method sets up a WiFi hotspot. The first parameter is the SSID name and the second one the inserted password.

 **myNumber** will store the value of the **my_phone_number** input as a string.
If the password is wrong the WiFi login interface will be restarted by resseting the ESP. 
  
- ### How to send and recevie messages from the ESP32 Feather to WhatsApp?

We first need to use the [ThingESP Arduino Library](https://github.com/SiddheshNan/ThingESP-Arduino-Library) in our Arduino IDE and create an account on the [following platform](https://thingesp.siddhesh.me/) and from there create a [new project](https://thingesp.siddhesh.me/#/console/projects). Afterwards we insert the following parameters inside the **thing()** constructor. Please use your own credentials to run your project as these are not valid.
 


```
ThingESP32 thing("Alexandru98", "Bathtub", "123456789");
```
1. Username
2. Project name
3. Project's Device Credentials


- ### How to save water level values into the ESP32 Feather flash memory?

We used a library called Preferences.h and it can save different data types onto the flash memory.
```
preferences.begin("App", false);
level1Set = preferences.getBool("level1_bool");
preferences.putBool("level1_bool", true);
```

With **preferences.begin()** we can initialize the **preferences** object with a certain set of values correlated with the **App** string. We can get and save various values related to the String **level1_bool**.

A better implementation variant would give us the possibility to have an unlimited number of levels that can be saved, this would be a possible future development.

* * * 

## The technical difficulties and how they were overcome
- Entering new Wi-Fi credentials on the device without connecting to a computer or manually modifying the code.

This was successfully achieved using the WiFiManager library

- Record the values ​​of the water levels so that they can be used after the device is rebooted.

This was successfully accomplished.  The values ​​are written to the flash memory using the Preferences.h library.

- Movement of the water and reflection on the walls of the container produce false measurements.

This was partially solved. Incorrect measurements because the reflected wave did not return to the sensor are recorded by the distance sensor as measurements with values ​​over 8 meters.  The sensor can only detect distances lower than 2 meters, so these incorrect measurements can be easily ruled out.

However, when the tub is filled the water oscillates which can cause incorrect measurements.  It can detect that the water is at a level that is not the real one.  This problem requires a deeper analysis which was not carried out in this project.

* * *

## Project value 
The value of the project lies in the simple fact that we proved that even after a small brainstorming session, one can still find challenges and difficulties that users encounter even in their everyday life. In our case it took around two days to identify various challenges and then agree on which one we chose to solve.

So after noticing that there was no solution similar to what we wanted, we set to work and brought it to fruition.


* * *

## **Demo** video
This is a demonstration that our user notification system is working properly! For the experiment we used:

- 2 water buckets: (one empty and one full). The empty bucket is used at the beginning to simulate an empty bathtub.
- Lamp holder: It is used to hold the sensor up and aimed at the bottom of the bucket.

* * *
 
https://user-images.githubusercontent.com/15964709/210858171-c8710c31-4560-43ef-b8ef-35b9d91e1f13.mp4


* * *
















<!-- Missing from the previous tutorial

❌ Sandro: General schema + Process

❌ Alex: Implementation 


* a schema of the concept of your project is missing in the tutorial. As is missing an electrical schema….Linking your presentation is not a solution. Take a look at your tutorial: would you be willing to read it without schema/electrical schema?
  ❌Sandro

* You should generalize your idea: why is this project interesting for programming arduino boards in general? Your intro should be much more interesting to invite the reader to engage in following the tutorial.  ✅ Sandro + Alexandru  

* The tutorial follows exactly the code. This is probably not the best option in terms of explaining your project (and even explaining the code).   ✅Alexandru 

* There is no presentation of a successful run (video?image?)       ✅Alexandru  -->