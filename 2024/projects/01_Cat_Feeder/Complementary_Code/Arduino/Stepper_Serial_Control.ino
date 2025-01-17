#include "Stepper.h"

// Define number of steps per revolution
const int stepsPerRevolution = 200;

// Give the motor control pins names
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8
#define dirA 12
#define dirB 13

// Initialise the stepper library on the motor shield
Stepper myStepper = Stepper(stepsPerRevolution, dirA, dirB);

void setup() {
  Serial.begin(9600);
  // Set the PWM and brake pins so that the direction pins can be used to control the motor
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);

  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  // Set the motor speed (RPMs):
  myStepper.setSpeed(60);
}


void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    // Move slightly forward and backward to prevent the feeder to get stuck
    myStepper.step(50);
    myStepper.step(-50);
    // Rotate the endless screw to feed the cat
    myStepper.step(300);

    delay(500);
  }
}