/*
 * ESP32 VEML7700 wifi reading
 * with automatuc updates (using ajax)
 * simplified from https://circuits4you.com/2018/11/20/web-server-on-esp32-how-to-update-and-display-sensor-values/
 */
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();


/* Put your SSID & Password */
const char* ssid = "<your wifi hotspots's name>";  // Change this value accordingly
const char* password = "<password>";  // your wifi hotspot's password


/* Put your MQTT borker credentials */
const char* mqttServer = "maqiatto.com";
const char* mqttUser = "<username>"; // your username on maqiatto.com
const char* mqttPassword = "<password>"; // your account's password on maqiatto.com
const char* mqttTopic = "<maqiatto username>/protofablab/lux";
char currentValue[10];

WiFiClient wlanClient;
PubSubClient mqttclient(wlanClient);

void setup(void){
  
  /* Serial */

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

   /* Wifi + server functions */

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

  
  /* MQTT setup */
  mqttclient.setServer(mqttServer, 1883);
  while (mqttclient.state() != 0) {
      Serial.println("Connecting to MQTT...");
      
      if (mqttclient.connect("ESP32Client", mqttUser, mqttPassword )) {
  
        Serial.println("connected");  
        //mmqttclient.setKeepAlive(1)
      } else {
  
        Serial.print("failed with state ");
        Serial.println(mqttclient.state());
        delay(2000);
  
      }
      delay(10);
  }
}


void loop(void){

  float luxValue = veml.readLux(VEML_LUX_AUTO);
  dtostrf(luxValue, 2, 2, currentValue);
  Serial.print("reading lux sensor: "); Serial.println(currentValue);
  if(mqttclient.publish(mqttTopic, currentValue)){
    Serial.println("Published");
  }

  delay(1000);

}
