# Arduino ESP32 S3 feather + IR emmiter and receiver

## Introduction

In this short tutorial we want to explain how to use the IR emmiter and IR receiver sensors that are compatible with the ESP32 S3 feather board.

Hence, we will go deeper into the code we wrote, here you can find [our presentation](https://docs.google.com/presentation/d/16dn7L52ekWoVPDxIX628QUl-f3-3H6wDVpz8NBfJ2Ys/edit#slide=id.p) which contains explanations with reference to the physical components used.

## From where this idea?

We started from the initial idea of being able to communicate between two computers via infrared beams, after a few challenges along the way we managed to achieve what we set out to do and we will explain in the following tutorial how we did it.

**Software Requirements**

1. [Arduino IDE](https://www.arduino.cc/en/software)
2. [IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote) library to be able to use functions to send and receive IR signals.
3. [LiquidCrystal_I2C.h](https://github.com/johnrickman/LiquidCrystal_I2C) library if your screen uses the I2C communications protocol.
4. [LiquidCrystal](https://github.com/arduino-libraries/LiquidCrystal) library if you don't have the I2C.

# Tutorial for the IR sender + IR receiver + LCD

### How to include the libraries?

We simply include them in the header of our code file, as in the example below.

```
#include <IRremote.h> // >v3.0.0 | tested on v3.9.0
#include <LiquidCrystal_I2C.h>
```

### Declaring constants

The declaration of constants will have to be done according to the connections we have between our Arduino board and the screen sensors we have. You will need to change the values of 11 or 10 accordingly.

```
#define PIN_RECV 11 // Pin connected to the IR receiver
#define PIN_SEND 10 // Pin connected to the IR emitter
```
