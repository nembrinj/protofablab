#define ROSSERIAL_ARDUINO_TCP // Very important!
#include "Adafruit_VEML7700.h"
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Float32.h>

const char *ssid = "the_ssid";
const char *password = "the_password";

Adafruit_VEML7700 veml = Adafruit_VEML7700();
IPAddress server(192, 168, 1, 3); // ip of your ROS machine (for Windows without Net Mirroring: the IP of Windows and you need to have port forwarding to redirect to your WSL2 linux)

ros::NodeHandle nh; // the node handler
std_msgs::Float32 output_val; // the type of message that will be sent
ros::Publisher pub("veml_lux", &output_val); // The publisher method; the first parameter corresponds to the name of topic on which to publish and the second the type of data

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){ // -> wifi delay -> a must -> otherwise it will restart continuously
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());

  // Setup VEML
  if (!veml.begin()) {
    Serial.println("VEML Sensor not found");
    while (1);
  }
  Serial.println("VEML Sensor found");
  
  // Setup connection to ROS machine (server)
  nh.getHardware()->setConnection(server, 11411);
  nh.initNode();
  nh.advertise(pub); // the topic must be advertised to the master
}

void loop(){
  //Send the value every second
  float veml_val = veml.readLux();
  output_val.data = veml_val;
  Serial.print("lux: "); Serial.println(veml_val); // for debug purposes
  pub.publish(&output_val); // publish the value on the topic
  nh.spinOnce(); // Required, otherwise topic not detected by ROS !
  delay(1000);
}
