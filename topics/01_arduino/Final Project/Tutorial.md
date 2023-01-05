# Tutorial for the HomeNotifier alongside the water sensor.



## Introduction
After a lot of brainstorming sessions in which we proposed, we came up with several projects and a few more discussions on the choice of project, this is the result. It took multiple coffee ☕, chocolate 🍫 and Friday (sometimes even Thursday) 🕑 breaks offered to the project to bring it to fruition.

### **Why** did we choose it?
We've been thinking about the different scenarios we're put in on a daily basis and have found out what we think can be a challenge ⚠️ for many of us. 

The basic idea is to eliminate the time it takes to constantly check the water level of a bathtub 🛀.

Suppose the following hypothetical situation: John 🧑 wants to take a bath in a bathtub, he is forced and forced to constantly check the water level so that it does not back 💦 up out of the bathtub.
This problem is completely eliminated if we use the water sensor together with HomeNotifier.

## General schema

El esquema del proyecto es modular. Cada modulo consiste en un sensor que se encuentra conectado a un ESP32 Feather. De acuerdo al sensor y a su uso, el ESP32 Feather estara programado para enviar y recibir datos via Wifi hacia el servidor de Twilio para luego ser transmitidos como mensajes 
de Whatsapp al usuario. 

//Figure slide 3

En este tutorial nos enfocamos en un tipo de uso que es el detectar el nivel del agua en una bañera a medida que esta se llena. Otros posibles usos pueden ser detectar la temperatura de una olla hasta que esta hierve.


//Figure slide 2

Para determinar los niveles del agua en la aplicación de la bañera hemos usado el sensor TOF VL53L0X. Este es un emisor y receptor de infrarrojos que mide el tiempo de vuelo entre que una pulsación parte del sensor y retorna reflejada. El sensor usa este tiempo para calcular la distancia que recorre el pulso. 

## Electrical schema



//Figure Electrical Schema

## Process



* * *
## **Demo** video
Aceasta este o demonstrație că sistemul nostru de notificare a utilizatorului funcționează corespunzător! Pentru experiment am folosit:

- 2 water buckets: (one empty and one full). The empty bucket is used at the beginning to simulate an empty bathtub.
- Lamp holder: It is used to hold the sensor up and aimed at the bottom of the bucket.


















❌ Sandro: General schema + Process

❌ Alex: Implementation 


* a schema of the concept of your project is missing in the tutorial. As is missing an electrical schema….Linking your presentation is not a solution. Take a look at your tutorial: would you be willing to read it without schema/electrical schema?
  ❌Sandro

* You should generalize your idea: why is this project interesting for programming arduino boards in general? Your intro should be much more interesting to invite the reader to engage in following the tutorial.  ✅ALEXANDRU 

* The tutorial follows exactly the code. This is probably not the best option in terms of explaining your project (and even explaining the code).  ❌ALEXANDRU 

* There is no presentation of a successful run (video?image?)       ❌ALEXANDRU 