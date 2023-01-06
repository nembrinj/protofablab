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

![Figure1](https://ucf7d26b82c39a6ffcaa8ad5715b.previews.dropboxusercontent.com/p/thumb/ABytSse8BDttl5yVXDi32gKEenvZkEvcy3oVCyBlNsiRsuYQ0FNRsrZf-ezfF1xsPkSb-E8hspzBYPO3nFGHz5tejDUO2TjsSkll7-H88CLp_8-4Z8LIb1Vd4vdP4OiFfNm0Wel47ynTn6bzZyYBxgyyiMo_qJrA5A9a-sp6X0JdQMM2BfCzsHrjbAI2BM_QWbMKURZaWpK-kLdpA93TXjK2SFKnc1mTgxn6ho6mtcNmWFe97NpSeUx58QVsvtfRqqkAc1yA5RMF0CabwAz0OUhNfcF-huKpNBei3k3QNNX_2OcSVDqZkDhPThvG_BnhdDyOwPOYqqgUE7T8FMBQp7BikXqwkU6p9oShh620aA1chTXlQHfCEUp18tqeBvgJlF8hU5fJ-ojoaRJjKbtlPN4e4D-BguKyXNZpiMeo7OEoSg/p.png)

In this tutorial, we focus on the application of detecting the water level in a bathtub as it fills up. Other possible uses may be to detect the temperature of a pot until it boils.

![Figure2](https://ucb7f89455edaa7604d7ce1cb1b5.previews.dropboxusercontent.com/p/thumb/ABwVLqOHD_VAjsOtcGIKHfINqapkPnnTYPJqQL76e_Dk_HA3EN3arUqAC2R6e0pGegIQ6gkbdbSztyRsJwxVV2SI_VgLYjOH25OfTuffiFon_X6Xze3X5nRCsXM3xvDcpg8Ysc2T3NvKYXCcF-cgVJ8OiEZGG7MXLOCnvS6Ez2K_WBWaiuWe6W-H2OoPBKKkgX9vnRRYRcAPcPErnJvA-eAiLBgwbc2NNdJiOXCRCjT7IGxFQHgK3e-elEwIbbdQe_lfXmubZMmhRp35W9y6lBq7JnB3bhlqCFMjMnIreGStLVwPkcVNqDCM4nkcYhkG6kzeXt1RJgJJARyOVxuuQFz6Js7B6cvNWPNlUht20dudsPUfFY5oxSntByHdaC4TZG5Pv_hPqltavkssWW6rsugw4zBe5AeqdImUqwTtB_2esA/p.png)

To determine the water levels in the bathtub application we have used the VL53L0X TOF sensor. This is an infrared emitter and receiver that measures the flight time that elapses between a pulse leaving the sensor and returning reflected. The sensor uses this time to calculate the distance at which the reflective surface is.

* * *

## Electrical schema

The connection between the TOF VL53L0X sensor and the ESP32 Feather is straightforward:

- GND in sensor to GND in ESP32 Feather
- Vin in sensor to GND in ESP32 Feather
- SCL in sensor to SCL in ESP32 Feather
- SDA in sensor to SDA in ESP32 Feather

![Figure3](https://uc47e333b6839944b9c5cd5d556c.previews.dropboxusercontent.com/p/thumb/ABx6PyepFs9GtezquKLWkJSyWGYefK30I51Oge8Fvto-pXmX85kGWtdTqxUZfVBLKvCpp-Ws0fdlVmoslAW3c7eMfUOhMjCmCsm4WElbVasZdcjjhMxZr89zMuKOhVrhRCeobe98LBOKvGSrxlflXIMyu_b1K0HBSPmWrACJWqSuj5z881hnlo8MmxyYjASR3CvDlRryu7URxK79sq4eql9EFhouwy08Wep8t9_uHCUz1pI68qIHDyfPMGl8kWesDbn1ivBM15hIETnoCvwZ2d_tC3r_kAnBJyVyPWnlwihv7Cs-DN0m_xHPhs3wzvzCn3WxewuCviGNYeW3Nq_xdRfcx0q-IgCmdxCLarNtCR2HeVJtQBgGW1XaarlrNYu48vNrm7KnaXslxf_4HeAUhrBjtxC7UL5L89IAZ0rNwVQ0wQ/p.png)
(Source: https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/python-circuitpython)


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
## **Demo** video
This is a demonstration that our user notification system is working properly! For the experiment we used:

- 2 water buckets: (one empty and one full). The empty bucket is used at the beginning to simulate an empty bathtub.
- Lamp holder: It is used to hold the sensor up and aimed at the bottom of the bucket.

* * *
 
https://user-images.githubusercontent.com/15964709/210858171-c8710c31-4560-43ef-b8ef-35b9d91e1f13.mp4



















<!-- Missing from the previous tutorial

❌ Sandro: General schema + Process

❌ Alex: Implementation 


* a schema of the concept of your project is missing in the tutorial. As is missing an electrical schema….Linking your presentation is not a solution. Take a look at your tutorial: would you be willing to read it without schema/electrical schema?
  ❌Sandro

* You should generalize your idea: why is this project interesting for programming arduino boards in general? Your intro should be much more interesting to invite the reader to engage in following the tutorial.  ✅ Sandro + Alexandru  

* The tutorial follows exactly the code. This is probably not the best option in terms of explaining your project (and even explaining the code).   ✅Alexandru 

* There is no presentation of a successful run (video?image?)       ✅Alexandru  -->