/*
 * ESP32 VEML7700 wifi reading
 * with automatuc updates (using ajax)
 * simplified from https://circuits4you.com/2018/11/20/web-server-on-esp32-how-to-update-and-display-sensor-values/
 */
#include <WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();


/* Put your SSID & Password */
const char* ssid = "ProFab";  // Enter SSID here
const char* password = "1700_UniFR.&";  //Enter Password here


/* Put your MQTT broker credentials */
const char* mqttServer = "raspijn.local";   // <------ CHANGE HERE
const int mqttPort = 1883;
//const char* mqttUser = "mqttuser";
//const char* mqttPassword = "password";
const char* mqttTopic = "esp32/lux";


WiFiClient wlanClient;
PubSubClient client(wlanClient);


bool LEDstatus = LOW;
char currentValue[10];

void setup(void){
  
  /* Serial */

  Serial.begin(115200);
  //while (!Serial) { delay(10); }
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

  client.setServer(mqttServer, mqttPort);
  
  while (!client.connected()) {
      Serial.println("Connecting to MQTT...");
  
      //if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      if (client.connect("ESP32Client")) {
  
        Serial.println("connected");  
  
      } else {
  
        Serial.print("failed with state ");
        Serial.println(client.state());
        delay(2000);
  
      }
  }

}


void loop(void){
  delay(1);
  
  if (millis()%2000 < 10) {
    float luxValue = veml.readLux(VEML_LUX_AUTO);
    dtostrf(luxValue, 2, 2, currentValue);
    Serial.print("reading lux sensor: "); Serial.println(currentValue);

    client.publish(mqttTopic, currentValue);
    LEDstatus = !LEDstatus;
  }

  if(LEDstatus)
    {digitalWrite(LED_BUILTIN, HIGH);}
  else
    {digitalWrite(LED_BUILTIN, LOW);}  
}