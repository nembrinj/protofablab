# Arduino ESP32 S3 feather + IR emmiter and receiver

## Introduction
In this short tutorial we want to explain how to use the IR emmiter and IR receiver sensors that are compatible with the ESP32 S3 feather board.

## Origins
#### Discovery of infrared 
In 1800, [William Herschel](https://science.nasa.gov/ems/07_infraredwaves) conducted an experiment measuring the difference in temperature between the colors in the visible spectrum. 

He placed thermometers within each color of the visible spectrum. The results showed an increase in temperature from blue to red. When he noticed an even warmer temperature measurement just beyond the red end of the visible spectrum, Herschel had discovered infrared light.

![Infrared](https://diyodemag.com/_images/5feb1eafc672e00d17dd9ab0,816,544)

#### Where is infrared light in the electromagnetic spectrum?
![Spectrum](https://www.fondriest.com/environmental-measurements/wp-content/uploads/2014/03/par_electromagnetic-spectrum.jpg)

As we can see, the sun is in the electromagnectic spectrum, which means that direct use of the IR receiving sensor may prove faulty if used directly outdoors without special filters.


### Introduction to IR Protocol

**[Advantages](https://www.konsyse.com/articles/infrared-communication-advantages-and-disadvantages/):**

1. Affordable
2. Easy to use
3. Power efficient

**Disadvantages:**

1. Limited Range
2. Unidirectional emitter
3. Can be obstructed

[IR receiver](https://create.arduino.cc/projecthub/electropeak/use-an-ir-remote-transmitter-and-receiver-with-arduino-1e6bc8) modules are used to receive IR signals. These modules work in 3,8 KHz frequency.

###   What is a pinout and GPIO ?   

- A pinout is a diagram showing the arrangement of pins on an integrated circuit and their functions.
- At the most basic level, GPIO refers to a set of pins on your computer’s mainboard or add-on card. These pins can send or receive electrical signals, but they aren’t designed for any specific purpose. This is why they’re called “general-purpose” IO.


###  This the pinout diagram for the ESP32 Feather

![Pinout](https://cdn-learn.adafruit.com/assets/assets/000/111/179/original/wireless_Adafruit_HUZZAH32_ESP32_Feather_Pinout.png?1651089809)


###   But what pins do we need to use in our case? 

![Imgur](https://imgur.com/TT1ZgET)

1. 3V pin
2. GND pin 
3. One digital pin (Number 6 in our case)

3V Pin - These pins are the output from the 3.3V regulator, they can supply 500mA peak.
GND Pin - This is the common ground for all power and logic.
From D5-D6, D9-D13 - These are digital pins. In our case, we used D6 for the communication between the Arduino board and the sensors.