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

The declaration of the constants (PIN_RECV and PIN_SEND) will have to be done according to the connections we have between our Arduino board and the screen sensors we have. You will need to change the values of 11 or 10 accordingly.

```
#define PIN_RECV 11 // Pin connected to the IR receiver
#define PIN_SEND 10 // Pin connected to the IR emitter
```

Then, the next step is to define the variable that will contain the character that we will send (input), the constant for the address that our sender will have (MY_ADDRESS).

```
char input;        // Related to the emitter
const uint16_t MY_ADDRESS = 0xAAAA; // Addresses must be different from the other emitter-receiver
```

### Instantiating the LiquidCrystal_I2C class

This can simply be done as described in the code below. In our case, the class LiquidCrystal_I2C takes 3 parameters, e.g. LiquidCrystal_I2C(I2Caddress, nrCols, nrRows).
In numerical order, this is what they are used for:

1. Address of IC2 module that is connected to Arduino. It can be found using the following [tutorial](https://lastminuteengineers.com/esp32-i2c-lcd-tutorial/), in our case is 0x27 (a code snippet can be found further down).
2. The number of columns that the display has.
3. The number of rows that the display has.

<details>
  <summary>Click here to get the code for Determining the I2C Address</summary>
  
  ### The code snippet
  ```#include <Wire.h>

void setup() {
Serial.begin (115200);

// Leonardo: wait for serial port to connect
while (!Serial)
{
}

Serial.println ();
Serial.println ("I2C scanner. Scanning ...");
byte count = 0;

Wire.begin();
for (byte i = 8; i < 120; i++)
{
Wire.beginTransmission (i);
if (Wire.endTransmission () == 0)
{
Serial.print ("Found address: ");
Serial.print (i, DEC);
Serial.print (" (0x");
Serial.print (i, HEX);
Serial.println (")");
count++;
delay (1); // maybe unneeded?
} // end of good response
} // end of for loop
Serial.println ("Done.");
Serial.print ("Found ");
Serial.print (count, DEC);
Serial.println (" device(s).");
} // end of setup

void loop() {}

```
</details>

`LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);`
```
