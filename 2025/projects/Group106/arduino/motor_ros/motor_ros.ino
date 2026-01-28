// Installing libraries: AccelStepper (Mike McCauley) and Rosserial Arduino Library (Michael Ferguson)
// 0.) Open Arduino IDE
// 1.) Go to Sketch → Include Library → Manage Libraries…
// 2.) In the Library Manager search box, type AccelStepper or Rosserial Arduino Library
// 3.) Click Install
// 4.) Once installed, restart the IDE and compile.

// Board:
// Adafruit Feather ESP32-S3 2MB PSRAM

#include <AccelStepper.h>
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Int16.h>

#define ROSSERIAL_ARDUINO_TCP
  
const char *ssid = "";
const char *password = "";

// Ip of your ROS machine
IPAddress server(192, 168, 1, 12);

int motorPin9 = 9;   // Driver: IN1
int motorPin10 = 10; // Driver: IN2
int motorPin11 = 11; // Driver: IN3
int motorPin12 = 12; // Driver: IN4

AccelStepper stepper(AccelStepper::FULL4WIRE, motorPin9, motorPin11, motorPin10, motorPin12);  // Note: order is NOT 9, 10, 11, 12

ros::NodeHandle nh; // the node
std_msgs::Int16 int_msg; // the type of message that will be received

void message_callback( const std_msgs::Int16& motor_speed){
  // Motor speed should be in RPM
  Serial.println("Received message: " + String(motor_speed.data));
  stepper.setSpeed(rpm_to_steps_per_sec(motor_speed.data));
}
ros::Subscriber<std_msgs::Int16> sub("motor_speed", &message_callback ); // topic to listen on


float rpm_to_steps_per_sec(const float rpm) {
  float stepsPerRev = 2048;  // WPI401 in full-step
  float stepsPerSec = rpm * stepsPerRev / 60.0;
  return stepsPerSec;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) // -> wifi delay -> a must -> other wise it will restart continuously↪
  { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());

  // Setup connection to ROS machine (server)
  nh.getHardware()->setConnection(server, 11512);
  nh.initNode();
  nh.subscribe(sub);

  stepper.setMaxSpeed(rpm_to_steps_per_sec(15));    // max RPM
  stepper.setSpeed(rpm_to_steps_per_sec(5));        // start speed
}

void loop() {
  nh.spinOnce(); // Required, otherwise topic not detected by ROS !
  stepper.runSpeed();  // moves one step at the current speed
}
