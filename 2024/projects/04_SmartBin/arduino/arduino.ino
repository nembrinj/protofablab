/************************************************************/
/*                 PREPROCESSOR DIRECTIVES                  */
/************************************************************/


#define ROSSERIAL_ARDUINO_TCP   // Required for using ROS over Arduino TCP
#define OPEN_ANGLE  80
#define CLOSE_ANGLE 0

#include <ESP32Servo.h>
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "Adafruit_VL53L0X.h"
#include <TimerEvent.h>

/************************************************************/
/*                       WIFI CREDENTIALS                   */
/************************************************************/
// Replace with your actual network credentials
const char* ssid     = "";
const char* password = "";

// IP of your ROS Master (if on Windows + WSL, forward port 11311 to WSL)
IPAddress server(192, 168, 178, 31);

/************************************************************/
/*                    GLOBAL VARIABLES                      */
/************************************************************/


// Servo
Servo myservo;

// ROS NodeHandle
ros::NodeHandle nh;


// Publisher: Publishes bin space status as a String
std_msgs::String binStatusMsg;
ros::Publisher binSpaceStatusPub("binSpaceStatus", &binStatusMsg);


// Subscriber: Receives servo commands ("OPEN" or "CLOSE")
String servoCommand;
bool isFull          = false;       // Indicates whether bin is "full"

// Timers
TimerEvent distanceCheckEvent;
const unsigned long distanceIntervalWhenClose = 3000;  // 3 seconds

// Push Button (with internal pull-up)
const byte buttonPin     = 5;
int buttonState          = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 1000; // 1 second

// VL53L0X Distance Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

// Debug/Loop timing (optional)
unsigned long previousMillis = 0;
unsigned long currentMillis  = 0;
unsigned long deltaTime      = 0;

/************************************************************/
/*                 ROS SUBSCRIBER CALLBACK                  */
/************************************************************/
/**
 * @brief  Called when a "servo_command" message arrives from ROS.
 *         Updates servo position (OPEN/CLOSE) unless bin is already full.
 *
 * @param  cmd_msg The incoming message of type std_msgs::String
 */
void servoCallback(const std_msgs::String& cmd_msg) {
  // If bin is already marked full, ignore further servo commands.
  if (isFull) {
    return;
  }

  servoCommand = cmd_msg.data;
  Serial.println(servoCommand);

  if (servoCommand == "OPEN") {
    Serial.println("Opening the lid");
    myservo.write(OPEN_ANGLE);
  } else if (servoCommand == "CLOSE") {
    Serial.println("Closing the lid");
    myservo.write(CLOSE_ANGLE);
  }
}

// ROS Subscriber object, listening on topic "servo_command"
ros::Subscriber<std_msgs::String> servoCmdSub("servo_command", &servoCallback);

/************************************************************/
/*               HELPER FUNCTION: GET DISTANCE              */
/************************************************************/
/**
 * @brief  Reads and returns the distance (in mm) from the VL53L0X sensor.
 *         Returns -1 if measurement is invalid.
 *
 * @return Distance in millimeters or -1 if invalid.
 */
int getDistanceInMillimeters() {
  lox.rangingTest(&measure, false);  // 'false' -> no debug prints
  if (measure.RangeStatus == 4) {
    // 4 = "phase failures," which indicates an invalid reading
    return -1;
  }
  return measure.RangeMilliMeter;
}

/************************************************************/
/*              TIMER CALLBACK: CHECK BIN STATUS            */
/************************************************************/
/**
 * @brief  Called periodically (every 3s) when the bin is closed.
 *         Checks if the bin is full (object detected <= 180 mm).
 */
void checkBinIfClosed() {
  // If lid is open, skip. We only check "fullness" if lid is closed.
  if (servoCommand == "OPEN") return;

  int distance = getDistanceInMillimeters();
  // If distance is invalid or not measured, do nothing
  if (distance < 0) return;

  // If an object is detected within 180 mm, mark bin as full
  if (distance <= 180) {
    isFull = true;
    binStatusMsg.data = "Full";
    binSpaceStatusPub.publish(&binStatusMsg);
    Serial.println("Bin is Full. Published status.");
  }
}

/************************************************************/
/*                           SETUP                          */
/************************************************************/
void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize ROS node (via server IP on port 11311)
  nh.getHardware()->setConnection(server, 11311);
  nh.initNode();
  nh.subscribe(servoCmdSub);
  nh.advertise(binSpaceStatusPub);

  // Attach the servo to pin 13
  myservo.attach(13);

  // Initialize the VL53L0X sensor
  Serial.println("Initializing VL53L0X sensor...");
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1); // Halt if sensor fails
  }
  Serial.println("VL53L0X Initialized.");

  // Set up the pushbutton pin with internal pull-up
  pinMode(buttonPin, INPUT_PULLUP);

  // Configure timer event to check the bin’s fullness every 3 seconds 
  distanceCheckEvent.set(distanceIntervalWhenClose, checkBinIfClosed);
}

/************************************************************/
/*                            LOOP                          */
/************************************************************/
void loop() {
  // Handle ROS communications
  nh.spinOnce();

  // Track time between loop iterations (optional, for debug)
  currentMillis = millis();
  deltaTime = currentMillis - previousMillis;
  previousMillis = currentMillis;

  //**********************************************************
  //                PUSH BUTTON DEBOUNCE CHECK
  //**********************************************************
  buttonState = digitalRead(buttonPin);
  unsigned long currentMillis = millis();
  if (buttonState == LOW && (currentMillis - lastDebounceTime) > debounceDelay) {
    // Toggle servo state on button press
    if (servoCommand == "OPEN") {
      myservo.write(CLOSE_ANGLE);
      servoCommand = "CLOSE";
      Serial.println("Closed by button press");
    } else {
      myservo.write(OPEN_ANGLE);
      servoCommand = "OPEN";
      Serial.println("Opened by button press");
    }
    lastDebounceTime = currentMillis;
  }

  //**********************************************************
  //                PERIODIC DISTANCE CHECK
  //**********************************************************
  distanceCheckEvent.update();  // Calls checkBinIfClosed() on schedule

  //**********************************************************
  //     IMMEDIATE DISTANCE CHECK IF BIN IS OPEN (TO CLOSE)
  //**********************************************************
  // If the bin is not full, and currently open, close if we detect an object within 180 mm.
  // (This allows immediate closure rather than waiting 3s)
  if (!isFull && servoCommand == "OPEN") {
    int distance = getDistanceInMillimeters();
    if (distance > 0 && distance <= 180) {
      myservo.write(CLOSE_ANGLE);
      servoCommand = "CLOSE";
      Serial.println("Closed by VL53L0X proximity");
    }
  }
}
