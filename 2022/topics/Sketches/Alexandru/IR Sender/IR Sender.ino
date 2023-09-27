#include <IRremote.h>

#define PIN_SEND 6


void setup() {
  // put your setup code here, to run once:
  IrSender.begin(PIN_SEND); // We initialize the IR sender
}

void loop() {
  // put your main code here, to run repeatedly:
  // IrSender.sendNEC(0x3D3D3D3D3D, 0x34, true, 0);
  IrSender.sendPronto("Sandro \0", 1);
  delay(1000);
}
