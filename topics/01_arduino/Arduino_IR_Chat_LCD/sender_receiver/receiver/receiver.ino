#include <IRremote.h> // >v3.0.0
                                           
#define PIN_RECV 5 // Pin connected to the IR receiver

void setup()  
{  
  Serial.begin(9600); //initialize serial connection to print on the Serial Monitor of the Arduino IDE
  IrReceiver.begin(PIN_RECV); // Initializes the IR receiver object
}  
                               
void loop()  
{  
  if (IrReceiver.decode()) {
    Serial.println("Received something...");    
    IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data. The command is received as HEX.
    Serial.println();
    Serial.println((char) IrReceiver.decodedIRData.command); // Converts the ASCII value received as command into a char and prints it.  
    Serial.println();
    IrReceiver.resume(); // Important, enables to receive the next IR signal
  }  
}  
