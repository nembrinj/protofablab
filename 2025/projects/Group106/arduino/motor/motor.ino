// This Arduino example demonstrates bidirectional operation of a
// 28BYJ-48, using a WPI401 - ULN2003 interface board to drive the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the
// operating Frequency is 100pps. Current draw is 92mA.

////////////////////////////////////////////////
//declare variables for the motor pins
int motorPin1 = 9; 
int motorPin2 = 10; 
int motorPin3 = 11;
int motorPin4 = 12;
// Red - 28BYJ48 pin 5 (VCC)
int motorSpeed = 1200; //variable to set stepper speed
int numSteps = 0;
int stepsPerRevolution = 512; // number of steps per full revolution
int lookup[8] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001};
//////////////////////////////////////////////////////////////////////////////
void setup() {
  //declare the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  Serial.begin(9600);
}

//////////////////////////////////////////////////////////////////////////////
void loop(){
  if(numSteps < stepsPerRevolution)
    clockwise();
  else
    anticlockwise();
  if (numSteps == stepsPerRevolution * 2)
    numSteps = 0;
  numSteps++;
}
//////////////////////////////////////////////////////////////////////////////
//set pins to ULN2003 high in sequence from 1 to 4
//delay "motorSpeed" between each pin setting (to determine speed)
void anticlockwise()
  {
    for(int i = 0; i < 8; i++)
    {
      setOutput(i);
      delayMicroseconds(motorSpeed);
    }
}
void clockwise()
{
  for(int i = 7; i >= 0; i--)
  {
    setOutput(i);
    delayMicroseconds(motorSpeed);
  }
}
void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(lookup[out], 0));
  digitalWrite(motorPin2, bitRead(lookup[out], 1));
  digitalWrite(motorPin3, bitRead(lookup[out], 2));
  digitalWrite(motorPin4, bitRead(lookup[out], 3));
}
