# DEUX-EX 
*by Henchoz Tristan & Peiris Ghamaathige*
## Table of Contents
- [DEUX-EX](#deux-ex)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [The Idea](#the-idea)
  - [Implementation](#implementation)
  - [Installation](#installation)
      - [Software](#software)
        - [Motor control unit](#motor-control-unit)
        - [Switch control unit](#switch-control-unit)
      - [Hardware](#hardware)
  - [Technical Difficulties](#technical-difficulties)
  - [Project Value](#project-value)
  - [Conclusion](#conclusion)



## Introduction

An electronic locking system could be useful in many situation, from managing a large number of user to remotly closing the windows in case of bad weather. But changing the whole lock system can be expensive and time consuming. DEUS-EX is a solution to get those advantage without having to change anything, as it uses the actual system in place and make it smarter.

## The Idea

Due to the new automated authentication system of the Learning lab door, this project suggests a physically connected new motorized door unlocking system. It includes three main aspects ; 
- Accepting and responding to the new authentication system, 
- Checking the current status of the door lock 
- Unlock/Lock the door respectively

Furthermore, this system enables a manual override mechanism during an emergency.

## Implementation

The implementation was done in 3 main steps:

1. Accepting and responding to the new authentication system

    An Arduino ESP32 feather connected to the learning lab Wifi will communicate with the new authentication system through MQTT messages. 
    Projected MQTT messages are as follows;

    | Message | Description | Returns |
    | - | - | - |
    | `flip_lock_state` | Change state of the door lock and return the new lock state | lock/unlock|
    | `get_current_lock_state` | Change the state of the door lock and return the new lock state | locked/unlocked|


2. Checking the current status of the door lock

    [Mini Microswitch SPDT (Roller Lever)](https://www.play-zone.ch/de/sparkfun-mini-microswitch-mit-roller-lever.html) which is triggered by the physical door lock is attached to the door by a new 3D printed piece that slides through to a side of the door. The readings of the microswitch are then sent to the ESP32 Feather.
    ![alt text](https://github.com/nembrinj/protofablab/blob/main/2023/projects/03_DEUS-EX/images/pic_lock_sensor.JPG)

3. Unlock/Lock the door respectively

    The signal to unlock/lock the door is then passed to an Arduino UNO R3. The motor, [NEMA17](https://gemsmotor.com/stepper-motor-manufacturer) is then connected to the Arduino UNO through a motor shield R3 is activated accordingly. Series of 3D-printed parts are connecting the door knob and the motor through a belt with the calculated tension to unlock/lock the door.

## Installation

#### Software

Required Technologies:

* [Arduino IDE](https://www.arduino.cc/)

Clone the repository :

 ```bash
 git clone https://github.com/nembrinj/protofablab.git
 ```

Move to the project DEUX EX directory :

```bash
cd 2023/projects/03_DEUX-EX
```

Move to the software code directory :

```bash
cd code
```

##### Motor control unit 

Open the **mortor_code.ino** in Arduino IDE and install the following libraries.

* [Stepper.h](https://www.arduino.cc/reference/en/libraries/stepper/)

Upload the relevant code to the Arduino UNO.

##### Switch control unit 

Open the **switch_control_with_mqtt.ino** in Arduino IDE and install the following libraries.

* [WiFi.h](https://www.arduino.cc/reference/en/libraries/wifi/)
* [PubSubClient.h](https://www.arduino.cc/reference/en/libraries/pubsubclient/)
* [ezButton.h](https://www.arduino.cc/reference/en/libraries/ezbutton/)

**``protofablab/2023/projects/03_DEUX-EX/code/switch_control_with_mqtt.ino``**

Enter the SSID and the password of the WIFI network in the following lines of the file.

```c++
const char* ssid = "";
const char* password = "";
```

Enter the details of the mqtt broker and user in the following lines of the file.

```c++
const char* mqttBroker = "";
const char* mqttClientName = "";
const String mqttUser = ""; 
const String mqttPass = "";
```

#### Hardware

As the main goal of this project is to interact with the physical world, it also requires a lot of differents hardware.
![alt text](https://github.com/nembrinj/protofablab/blob/main/2023/projects/03_DEUS-EX/images/pic_general.JPG)

On the electronics side, this project uses an Arduino Uno R3 with an Arduino Motorshield R3, a esp32 Feather and a stepper motor. This is connected to the door using a belt and some 3D printed pieces.

The 3D pieces are located in the folder [3D_objects](https://github.com/nembrinj/protofablab/blob/main/2023/projects/03_DEUS-EX/3D_objects) and require the following:

* [OpenSCAD](https://openscad.org/)
* [BOSL library](https://github.com/revarbat/BOSL)
* [3D printer (Original Prusa i3 MK3S+)](https://www.prusa3d.com/category/original-prusa-i3-mk3s/)



## Technical Difficulties

The technical difficulties faced during the project can be categorized;

- **Design of the system**

    Initially, it was difficult to get a proper idea of designing the prototype. However, after referring to a few similar systems we began outlining the prototype.

- **3D printing difficulties**

    The precise measurements of the physical components did not meet the final expectations of the 3D printed components as it had errors which we did not manage to calculate and the printer has limitations with the minimum thickness.
    The strength of the 3D-printed components was not able to cope with the tension of the belt and system.
    The time to 3D print some components was relatively high.

- **Choosing the accurate technologies** 

    The Nema17 motor required 12V inputs but the Arduino Feather outputs were 5V, So we were unable to integrate the switch and the motor shield in one microcontroller. We had to connect Motorsheild to an Arduino Uno which was not Wifi compatible.


## Project Value

- **Interaction between real world and IT** 
![alt text](https://github.com/nembrinj/protofablab/blob/main/2023/projects/03_DEUS-EX/images/demo.mp4)

## Conclusion

The DEUS-EX project featuring a door unlocking system of the Learning Lab enables a new automated unlocking system. With several distinct features, this project allows for automated system without having to change any existing setup.


