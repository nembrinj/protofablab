/*
  The circuit:
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 6
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

const uint16_t MY_ADDRESS = 0xBBBB; // Addresses must be different from the other emitter-receiver


// include the library code:
#include <IRremote.h> // >v3.0.0 | tested on v3.9.0
#include <LiquidCrystal.h>

#define PIN_RECV 10 // Pin connected to the IR receiver
#define PIN_SEND 11 // Pin connected to the IR emitter
char input;        // Related to the emitter


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int col = 0, row = 0;
char letter = "";


void setup() {
  // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    // Print a message to the LCD.
    //lcd.print("hello, world!");

  // START CODE FOR THE RECEIVER
    Serial.begin(9600); //initialize serial connection to print on the Serial Monitor of the Arduino IDE
    IrReceiver.begin(PIN_RECV); // Initializes the IR receiver object
  // END CODE OF THE RECEIVER

  // START CODE FOR THE SENDER
    IrSender.begin(PIN_SEND); // Initializes IR sender
    delay(1000);
    Serial.println("Type something!");
  // END CODE FOR THE SENDER
}

void loop() {

  // START CODE FOR THE RECEIVER
   if (IrReceiver.decode()) {
    if( !(IrReceiver.decodedIRData.address == MY_ADDRESS) ){
      Serial.println("Received something...");    
      IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data. The command is received as HEX.
      Serial.println();
      letter = (char) IrReceiver.decodedIRData.command; // Converts the ASCII value received as command into a char and prints it.  
      Serial.println(letter); 

      lcd.setCursor(col, row);
      lcd.print(letter);
      col++;
      if (row == 1 && col == 15){
        delay(1000);
        lcd.clear();
        row = 0;
        col = 0;
      }
      if (col == 15){
        row++;
        col = 0;
      }
    }
    Serial.println();
    IrReceiver.resume(); // Important, enables to receive the next IR signal
  }
  // END CODE OF THE RECEIVER

  // START CODE FOR THE SENDER
  if(Serial.available()){ // Checks if something is written in the serial monitor
      input = Serial.read();
      Serial.print("You typed: " );
      Serial.println(input);
      IrSender.sendNEC(MY_ADDRESS, input, 0); // the address 0xFE01D6DE with the command input and 0 repetitions is sent. 
      delay(60);    
  }

  // END CODE FOR THE SENDER

}

