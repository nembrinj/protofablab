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
const char* ssid = "ProFab";  // Enter SSID here
const char* password = "1700_UniFR.&";  //Enter Password here


WiFiClient wlanClient;

WebServer server(80);

bool LEDstatus = LOW;
char currentValue[10];


/* index page */
void handleINDEX() {
  String s = MAIN_page; //Read HTML contents
  server.send(200, "text/html", s); //Send web page
}

;void handleLUX() { 
  //server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plane", currentValue); // send lux value to client ajax request 
}



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


  /* Webserver setup */

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
  
  if (millis()%2000 < 10) {
    float luxValue = veml.readLux(VEML_LUX_AUTO);
    dtostrf(luxValue, 2, 2, currentValue);
    Serial.print("reading lux sensor: "); Serial.println(currentValue);

    LEDstatus = !LEDstatus;
  }

  if(LEDstatus)
    {digitalWrite(LED_BUILTIN, HIGH);}
  else
    {digitalWrite(LED_BUILTIN, LOW);}  
}
