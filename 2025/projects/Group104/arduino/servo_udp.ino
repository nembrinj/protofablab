#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

const char* ssid     = "WIFI";
const char* password = "password";

WiFiUDP udp;
const int localPort = 4210;

// Servo
Servo servo1;
const int servoPin = 6;      // pin of servo
// Servo state
float pos = 90.0;         // initial position
float vel = 0.0;          // speed received
float pos_min = 90.0;      // min position
float pos_max = 180.0;    // max position

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  servo1.attach(servoPin);
  servo1.write((int)pos);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.print("Listening on UDP port ");
  Serial.println(localPort);

  lastTime = millis();
}

void loop() {
  // time passed
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // seconds
  lastTime = now;

  // update position constantly based on last velocity
  pos += vel * dt;

  // clip position
  if (pos < pos_min) pos = pos_min;
  if (pos > pos_max) pos = pos_max;

  // move servo
  servo1.write((int)pos);

  Serial.print("Current pos: ");
  Serial.println(pos);

  // check for UDP message
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buffer[32];
    int len = udp.read(buffer, 32);
    if (len > 0) buffer[len] = 0; // null-terminate

    float newVel = atof(buffer); // cast to float
    vel = newVel;

    Serial.print("Received vel (deg/s): ");
    Serial.println(vel);
    
  }

  delay(1);
}