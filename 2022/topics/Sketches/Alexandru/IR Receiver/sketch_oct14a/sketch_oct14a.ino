#include <IRremote.h>

const int IR_RECEIVER_PIN = 5;
const int ledPin = LED_BUILTIN;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVER_PIN);
}

void loop() {

  if(IrReceiver.decode()){
    Serial.print(" " + IrReceiver.decodedIRData.command);
    IrReceiver.resume(); // Receive the next value
  }
  delay(600); 
  
}
