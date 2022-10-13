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
const char* ssid = "my_raspi_network";  // Enter SSID here
const char* password = "password1234";  //Enter Password here

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
 
  //server.sendHeader("Access-Control-Allow-Origin", "*");
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
  WiFi.mode(WIFI_STA); //Connect to wifi
  WiFi.begin(ssid, password);

  Serial.println("Connecting to ");
  Serial.print(ssid);

  //Wait for WiFi to connect
  while(WiFi.waitForConnectResult() != WL_CONNECTED){      
      Serial.print(".");
    }
    
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP


 
  Serial.println("Starting server...");
  
  server.on("/", handleINDEX);      // index page
  server.on("/readlux", handleLUX);// only for readng values
  server.enableCORS(true);
  
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
