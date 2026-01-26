#define ROSSERIAL_ARDUINO_TCP

#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>

// Wifi
const char *ssid = "Coolheits_Oase";
const char *password = "SwaggerDagger";

// IP of ROS machine
IPAddress server(10, 233, 108, 195);
const int ros_port = 11411;

// Relay pin
#define RELAY_PIN 13

// ROS
ros::NodeHandle nh;

// Motor state tracking
bool motor_on = false;

// Callback
void motorCb(const std_msgs::String &msg)
{
  if (strcmp(msg.data, "on") == 0)
  {
    digitalWrite(RELAY_PIN, HIGH);
    motor_on = true;
    Serial.println("Motor ON");
  }
  else if (strcmp(msg.data, "off") == 0)
  {
    digitalWrite(RELAY_PIN, LOW);
    motor_on = false;
    Serial.println("Motor OFF");
  }
  else
  {
    Serial.println("Unknown command");
  }
}

// Subscriber
ros::Subscriber<std_msgs::String> sub("dc_motor_cmd", motorCb);


void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // motor off by default

  Serial.begin(115200);

  // WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // ROS
  nh.getHardware()->setConnection(server, ros_port);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
