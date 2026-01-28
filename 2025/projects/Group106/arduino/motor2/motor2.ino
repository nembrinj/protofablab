#include <Stepper.h>

int motorPin9 = 9;   // Driver: IN1
int motorPin10 = 10; // Driver: IN2
int motorPin11 = 11; // Driver: IN3
int motorPin12 = 12; // Driver: IN4

int stepsPerRevolution = 2048; // number of steps per full revolution

Stepper myStepper(stepsPerRevolution, motorPin9, motorPin11, motorPin10, motorPin12);  // Note: order is NOT 9, 10, 11, 12

// 10 and 15 works, 20 is too high.
// 15: one revolution takes 4 seconds. 
int rpm = 15;

void setup() {
  myStepper.setSpeed(rpm);
  Serial.begin(9600);
}

void loop(){
  // Note: .step() is a blocking call -- it will execute the number of steps specified and then return.

  Serial.println("Clockwise rotation");
  myStepper.step(stepsPerRevolution);   // one full rotation CW.
  delay(2000);

  Serial.println("Counterclockwise rotation");
  myStepper.step(-stepsPerRevolution);  // one full rotation CCW
  delay(2000);
}

