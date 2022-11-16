# Arduino ESP32 S3 feather + IR emmiter and receiver

## Introduction
In this short tutorial we want to explain how to use the IR emmiter and IR receiver sensors that are compatible with the ESP32 S3 feather board.

Hence, we will go deeper into the code we wrote, here you can find [our presentation](https://docs.google.com/presentation/d/16dn7L52ekWoVPDxIX628QUl-f3-3H6wDVpz8NBfJ2Ys/edit#slide=id.p) which contains explanations with reference to the physical components used. 

## From where this idea?
We started from the initial idea of being able to communicate between two computers via infrared beams, after a few challenges along the way we managed to achieve what we set out to do and we will explain in the following tutorial how we did it.

### 2 code bases
There will be some differences in the code presented because the screen that does not use the I2C interface was developed by Sandro Joel Hernández Goicochea and the one that uses the I2C interface was developed by Alexandru Filipescu. These changes are not major.

**Software Requirements**
1. [Arduino IDE](https://www.arduino.cc/en/software)
2. [IRremote](https://github.com/Arduino-IRremote/Arduino-IRremote) library to be able to use functions to send and receive IR signals.
3.  [LiquidCrystal_I2C.h](https://github.com/johnrickman/LiquidCrystal_I2C) library if your screen uses the I2C communications protocol.
4.  [LiquidCrystal](https://github.com/arduino-libraries/LiquidCrystal) library if you don't have the I2C.



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
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}  // end of setup

void loop() {}
  ```
</details>

`LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);`
* * *
### The setup() function
The **setup**() function is called when running the sketch, it is used to initialize variables, pin usage modes and libraries usage. This function is executed only once, after rebooting or resetting the Arduino. More about it can be found [here](https://arduinogetstarted.com/reference/arduino-setup).

```
void setup() {
    // START CODE FOR THE RECEIVER
    Serial.begin(9600); //initialize serial connection to print on the Serial Monitor of the Arduino IDE
    IrReceiver.begin(PIN_RECV); // Initializes the IR receiver object
    // END CODE OF THE RECEIVER

    // START CODE FOR THE SENDER
    IrSender.begin(PIN_SEND); // Initializes IR sender

    lcd.init();
    lcd.backlight();
    
    delay(1000);
    Serial.println("Type something!");
    // END CODE FOR THE SENDER
}
```
We will list the contents of the **setup()** function in descending order below:
- `Serial.begin(9600)` According to the official [documentation](https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/), it sets the data rate in bits per second (baud) for serial data transmission. For communicating with Serial Monitor, make sure to use one of the baud rates listed in the menu at the bottom right corner of its screen. We use the number **9600** becasuse we will look for this rate in the serial monitor.
- `IrReceiver.begin(PIN_RECV)` We use the static method **begin()** from the class **IrReceiver**, we have to input in the **begin()** function the number of the data pin for the IR receiver.
- `IrSender.begin(PIN_SEND)` The same as before but for the IR sender.
- `lcd.init()` This [function](https://arduinogetstarted.com/tutorials/arduino-lcd-i2c) addresses the lcd screen to initialize it.
- `lcd.backlight()` This function turns on the backlight.
- `delay(1000)` It allows us to pause the execution for our Arduino program for a specified period of time, in our case is 1000ms or 1 second.

* * *


### Inside the loop() function

**Initially we take care of the code for the receiver** 
```
void loop() {
  // START CODE FOR THE RECEIVER
   if (IrReceiver.decode()) {

    if( !(IrReceiver.decodedIRData.address == MY_ADDRESS) ){
      Serial.println("Received something...");    
      IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data. The command is received as HEX.
      letter = (char)IrReceiver.decodedIRData.command;
      
      lcd.setCursor(col,row);
      lcd.print(letter);
      col++;
      if(col > 15){
        col = 0;
        row++;
      }
      if(row > 1){
        row = 0;
        lcd.clear();
      }  
    }
    
    Serial.println();
    IrReceiver.resume(); // Important, enables to receive the next IR signal
  }
```

- `IrReceiver.decode()` Is used to verify if new IR commands were received.
- `!(IrReceiver.decodedIRData.address == MY_ADDRESS)` Here we check that the decoded address does not belong to our sender, so that only messages from an external sender are displayed.
- `letter = (char)IrReceiver.decodedIRData.command;` Converts the ASCII value received as command into a char, afterwards we save it in the variable **letter**.
- `lcd.setCursor(col,row)` We move the cursor to the desired position (column_index, row_index).
- `lcd.print(letter)` Here we print the desired letter on the LCD screen on the previously set position.
-  ```&nbsp 
	  col++;
      if(col > 15){
        col = 0;
        row++;
      }
      if(row > 1){
        row = 0;
        lcd.clear();
      }  
This is a simple set of instructions that will increment the columns variable, **the first if()** is entered if the **col** variable, has exceeded the total number of columns (by index) of our screen, then it resets the **col** variable and increments the **row** variable that represents the row we are on the LCD screen.

In **the second if()** we check if we have exceeded the maximum number of rows supported by the screen, if we have exceeded the limit (in our case index 1), we **reset the rows** and **clear the screen** of text with the command **lcd.clear()**.
- `IrReceiver.resume()` As commented, it enables receiving of the next value.


#### The sender code
```
    if(Serial.available()){ // Checks if something is written in the serial monitor
      input = Serial.read();
      Serial.print("You typed: " );
      //Serial.println(input);
      IrSender.sendNEC(MY_ADDRESS, input, 0);

      delay(60); 
  }
```
- `if(Serial.available())` As mentioned in the comment, we verify if something is written in the serial monitor by us.
- `input = Serial.read()`  Serial.read() will return the first value available in the serial receive buffer, so it will save the typed character inside input.
- `IrSender.sendNEC(MY_ADDRESS, input, 0)` As we want to use the NEC protocol we use the **sendNEC()** function. In the **first** parameter (MY_ADDRESS) we insert the predefined address of our Arduino, in the **second** parameter we just place the **input** which is the char typed from the keyboard and lastly we input the number of repetitions for the message which is 0 in our case.
- `delay(60)` Even though at the end but with **particular importance**, we place a delay of 60 milliseconds. In our experiments, the code did not work with a 50ms delay, we even encourage using a higher number such as 70ms. These numbers are **not random**, the NEC protocol needs **67.5 ms** in total to be received correctly.

