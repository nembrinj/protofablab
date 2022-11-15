# Arduino ESP32 S3 feather + IR emmiter and receiver

## Introduction

In this short tutorial we want to explain how to use the IR emmiter and IR receiver sensors that are compatible with the ESP32 S3 feather board.

Hence, we will go deeper into the code we wrote, here you can find [our presentation](https://docs.google.com/presentation/d/16dn7L52ekWoVPDxIX628QUl-f3-3H6wDVpz8NBfJ2Ys/edit#slide=id.p) which contains explanations with reference to the physical components used.

## From where this idea?

We started from the initial idea of being able to communicate between two computers via infrared beams, after a few challenges along the way we managed to achieve what we set out to do and we will explain in the following tutorial how we did it.

### Libraries needed

[IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote) and [LiquidCrystal_I2C.h](https://github.com/johnrickman/LiquidCrystal_I2C) for if your screen uses the I2C communications protocol and [LiquidCrystal](https://github.com/arduino-libraries/LiquidCrystal) library if you don't have the I2C.
