#include <IRremote.h> // >v3.0.0
                                            
#define PIN_SEND 6 // Pin connected to the IR emitter
char input;

void setup()  
{  
  IrSender.begin(PIN_SEND); // Initializes IR sender
  Serial.begin(9600);
  delay(1000);
  Serial.println("Type something!");
}  
                               
void loop()  
{  
  
    if(Serial.available()){ // Checks if something is written in the serial monitor
        input = Serial.read();
        Serial.print("You typed: " );
        Serial.println(input);
        IrSender.sendNEC(0xFE01D6DE, "abcd", 0); // the address 0xFE01D6DE with the command input and 0 repetitions is sent. 
    }

  delay(1000); // wait for one second
}  
