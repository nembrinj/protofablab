/*
 * ESP32 VEML7700 wifi reading
 * with automatuc updates (using ajax)
 * simplified from https://circuits4you.com/2018/11/20/web-server-on-esp32-how-to-update-and-display-sensor-values/
 */
#include <WiFi.h>
#include <WebServer.h>

#include "index.h"  //Web page content in header file -> PROGMEM

#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();


/* Put your SSID & Password */
const char* ssid = "protofab";  // Enter SSID here
const char* password = "asdfqwer1234";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);
bool LEDstatus = LOW;


/* index page */
void handleINDEX() {
  String s = MAIN_page; //Read HTML contents
  server.send(200, "text/html", s); //Send web page
}
 
void handleLUX() {
  float a = veml.readLux(VEML_LUX_AUTO);
  String luxValue = String(a);
  Serial.print("reading lux sensor: "); Serial.println(luxValue);
 
  server.send(200, "text/plane", luxValue); // send lux value to client ajax request 
  LEDstatus = !LEDstatus;
}

void setup(void){
  
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Wifi VEML7700 Test");

  /* VEML functions */
  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");
  
  pinMode(LED_BUILTIN, OUTPUT);

  /* wifi + server functions */
  Serial.println();
  Serial.println("Starting Wifi...");

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
 
  Serial.println("Starting server...");
  
  server.on("/", handleINDEX);      // index page
  server.on("/readlux", handleLUX);// only for readng values
 
  server.begin();                  //Start server
  
  Serial.println("Server started");
}


void loop(void){
  server.handleClient();
  delay(1);
  
  if(LEDstatus)
    {digitalWrite(LED_BUILTIN, HIGH);}
  else
    {digitalWrite(LED_BUILTIN, LOW);}  
}
