/*********
  Simon Ruffieux
  This example has been simplified from https://RandomNerdTutorials.com/esp32-servo-motor-web-server-arduino-ide/
  Originally based on the ESP32Servo Sweep Example
*********/

#include <ESP32Servo.h>

static const int servoPin = 6;

Servo servo1;

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin);
}

void loop() {
  for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }
}