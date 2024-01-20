/*
 * This code is use in the demo video 
 * https://github.com/nembrinj/protofablab/blob/main/2023/projects/03_DEUS-EX/images/demo.mp4
 */

// Include the Stepper library
#include  <Stepper.h>

// Define number of steps per revolution
const int stepsPerRevolution = 400;

// Give the motor control pins names
#define pwmA 3
#define pwmB 11
#define brakeA 9
#define brakeB 8
#define dirA 12
#define dirB 13

// Define motor direction
#define closing -1
typedef short int direction;
direction task = closing;
int objective = 0;

// Define the control pin
#define indoor 5

// Initialize the stepper library on the motor shield
Stepper myStepper = Stepper(stepsPerRevolution, dirA, dirB);

void setup() {
  // Set the PWM and brake pins so that the direction pins can be used to control the motor
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);

  // Set the door pin
  pinMode(indoor, INPUT);

  // Set default value
  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);

  // Set the motor speed (RPMs)
  myStepper.setSpeed(60);
}

void loop() {
  delay(7000);
  task = task * (-1);
  objective = objective + task;
  //mean 0 -> 1 -> 0 

  // HIGH is open
  int doorstate = digitalRead(indoor);
  while (doorstate != objective) {
    turn(task);
    doorstate = digitalRead(indoor);
  }

  turn(task*6);
}

void turn(direction d) {
  myStepper.step(-d * 25);
  delay(5);
}