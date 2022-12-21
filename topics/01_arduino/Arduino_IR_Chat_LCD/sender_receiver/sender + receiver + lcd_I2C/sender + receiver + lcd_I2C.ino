#include <IRremote.h> // >v3.0.0 | tested on v3.9.0
#include <LiquidCrystal_I2C.h>




#define PIN_RECV 11 // Pin connected to the IR receiver
#define PIN_SEND 10 // Pin connected to the IR emitter

char input;        // Related to the emitter

const uint16_t MY_ADDRESS = 0xAAAA; // Addresses must be different from the other emitter-receiver

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); 

int col = 0, row = 0;
char letter = "";

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

void loop() {
  // START CODE FOR THE RECEIVER
   if (IrReceiver.decode()) {

    if( !(IrReceiver.decodedIRData.address == MY_ADDRESS) ){
      Serial.println("Received something...");    
      IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data. The command is received as HEX.
      letter = (char)IrReceiver.decodedIRData.command;
      //Serial.print(letter); // Converts the ASCII value received as command into a char and prints it.
      
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
  // END CODE OF THE RECEIVER

  // START CODE FOR THE SENDER
    if(Serial.available()){ // Checks if something is written in the serial monitor
      input = Serial.read();
      Serial.print("You typed: " );
      //Serial.println(input);
      IrSender.sendNEC(MY_ADDRESS, input, 0); // the address 0xFE01D6DE with the command input and 0 repetitions is sent. 

      delay(60); 
  }

  // wait for one second
  // END CODE FOR THE SENDER
}
