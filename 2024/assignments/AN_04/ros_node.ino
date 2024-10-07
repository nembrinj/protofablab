#define ROSSERIAL_ARDUINO_TCP   // Very important ! 
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
 
const char *ssid = "ssid";
const char *password = "password";
 
IPAddress server(192, 168, 1, 3); // ip of your ROS machine (for Windows, the IP of Windows and you need to have port forwarding to redirect to your WSL2 linux)
 
ros::NodeHandle nh; // the node
std_msgs::String str_msg; // the type of message that will be sent
ros::Publisher pub("chatter", &str_msg); // the topic on which to publish the data (parameters: name, type of message)
 
void setup()
{ 
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) // -> wifi delay -> a must -> other wise it will restart continuously 
  { delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected - IP address:  ");
  Serial.println(WiFi.localIP());
 
  // Setup connection to ROS machine (server)
  nh.getHardware()->setConnection(server,11411);
  nh.initNode();
  nh.advertise(pub); // the topic must be advertised to the master
}
 
void loop(){
  //Send a HelloWorld message every second
  std_msgs::String str;
  str.data = "hello world";
  Serial.println("Sending message");
  pub.publish( &str );
  nh.spinOnce(); // Required, otherwise topic not detected by ROS !
  delay(1000); 
}
