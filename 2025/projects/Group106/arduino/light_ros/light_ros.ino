// Board: ESP32C3 Dev Module

#define ROSSERIAL_ARDUINO_TCP
#define LED_PIN 10

#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

const char *ssid = "";
const char *password = "";

// ip of your ROS machine (for Windows, if you use port
// forwarding, it is the IP of Windows and you need to have port forwarding to redirect
// packets to your WSL linux)
IPAddress server(192, 168, 1, 12); 

ros::NodeHandle nh; // the node
std_msgs::Int16 int_msg; // the type of message that will be received

// -------- STATE --------
volatile float freq_hz = 1.0;       // Blink frequency (Hz)
volatile int intensity = 255;       // LED brightness (0–255)
unsigned long lastToggle = 0;
bool ledOn = false;

void intensity_callback( const std_msgs::Int16& msg){
  Serial.println("Received intensity");
  intensity = constrain(msg.data, 0, 255);
}
void frequency_callback( const std_msgs::Int16& msg){
  Serial.println("Received frequency");
  if (msg.data >= 0) freq_hz = msg.data;
}
ros::Subscriber<std_msgs::Int16> subInt("light_intensity", &intensity_callback ); // topic to listen on
ros::Subscriber<std_msgs::Int16> subFreq("light_frequency", &frequency_callback ); // topic to listen on

void setup()
{

  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) // -> wifi delay -> a must -> otherwise it will restart continuously↪
  { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(LED_PIN, OUTPUT);

  // Setup connection to ROS machine (server)
  nh.getHardware()->setConnection(server, 11511);
  nh.initNode();
  nh.initNode();
  nh.subscribe(subInt);
  nh.subscribe(subFreq);
}

void loop()
{
  nh.spinOnce();

  if (freq_hz == 0) {
    analogWrite(LED_PIN, intensity); 
    return;
  }

  // Compute ON/OFF toggle timing
  float halfPeriod = (1000.0 / freq_hz) / 2.0;  // ms
  unsigned long now = millis();

  if (now - lastToggle >= halfPeriod) {
    ledOn = !ledOn;
    lastToggle = now;

    if (ledOn) {
      analogWrite(LED_PIN, intensity);  // LED ON → set brightness
    } else {
      analogWrite(LED_PIN, 0);          // LED OFF
    }
  }
}
