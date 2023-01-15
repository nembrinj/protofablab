/*
 * @Descripttion: 
 * @version: 
 * @Author: Elegoo
 * @Date: 2020-06-04 11:42:27
 * @LastEditors: COSTA Kimmy and MEMBREZ Cédric
 * @LastEditTime: 2023-01-15
 */
#include "CameraWebServer_AP.h"
#include <WiFi.h>
#include "esp_camera.h"
#include <PubSubClient.h>
#include <WebSocketsClient.h>      // send data (image buffer) to PiZero

WiFiClient wifiClient;
PubSubClient client(wifiClient);

#define RXD2 33
#define TXD2 4
CameraWebServer_AP CameraWebServerAP;
bool WA_en = false;

/* web socket */
WebSocketsClient webSocket;
#define USE_SERIAL Serial1

String messageMQTT = "";

/* WiFi Settings */
const char* ssid = "Pixel_5164";        // "OnePlus 8T Kimmy";   //!!!!!!!!!!!!!!!!!!!!!
const char* password = "0988027bc02c";  // "9e4abdc003dd0";       //!!!!!!!!!!!!!!!!!!!!!

///
/// Connect to wifi "ssid" using "password"
///
void wifiConnection(){
  WiFi.mode(WIFI_STA);     // connect to wifi
  WiFi.begin(ssid, password);

  // wait for WiFi to connect
  while(WiFi.waitForConnectResult() != WL_CONNECTED){
    Serial.print(".");
  }

  // If successful
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

/* MQTT settings */
const char* mqtt_server = "192.168.15.54";
const int mqtt_port = 1883;
const char* mqtt_topic = "teddyCtrl";

///
/// Print the payload on the topic.
///
void callback(char* topic, byte* payload, unsigned int length){

  String totalMessage = "";

  totalMessage += "{\"";
  totalMessage += topic;
  totalMessage += "\":";

  for (int i=0;i<length;i++) {
    char receivedChar = (char)payload[i];
    totalMessage += receivedChar;
  }
  totalMessage += "}";
  Serial.println(totalMessage);
  Set_MessageMQTT(totalMessage);
}

void Set_MessageMQTT(String message){
  messageMQTT = message;
}


///
/// Connect and Subscribe to MQTT
///
void reconnect() {
 // Loop until we're reconnected
 Serial.println("MQTT: reconnect ................");
 
 while (!client.connected()) {
   Serial.print("Attempting MQTT connection...");
   // Attempt to connect
   if (client.connect("ESP32Client")) {
     Serial.println("connected");
     // ... and subscribe to topic
     client.subscribe(mqtt_topic);
   } else {
     Serial.print("failed, rc=");
     Serial.print(client.state());
     Serial.println(" try again in 5 seconds");
     // Wait 5 seconds before retrying
     delay(5000);
   }
 }
}

/* Camera capture and transmission */

///
/// Capture a still image
///
void captureStillImage(){
  camera_fb_t* frameBuffer = esp_camera_fb_get();

  if(frameBuffer == NULL){
    Serial.println("frameBuffer is NULL");
  }

  webSocket.sendBIN(frameBuffer->buf, frameBuffer->len);

  esp_camera_fb_return(frameBuffer);
}

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	USE_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			USE_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		USE_SERIAL.printf("%02X ", *src);
		src++;
	}
	USE_SERIAL.printf("\n");
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			USE_SERIAL.printf("[WSc] Disconnected!\n");
			break;
		case WStype_CONNECTED:
			USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

			// send message to server when Connected
			webSocket.sendTXT("Connected");
			break;
		case WStype_TEXT:
			USE_SERIAL.printf("[WSc] get text: %s\n", payload);

			// send message to server
			break;
		case WStype_BIN:
			USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
			hexdump(payload, length);

			// send data to server
			break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;

    default:
      USE_SERIAL.printf("type not handled");
      break;
	}

}

/* Arduino setup and loop */
///
/// Connect to WiFi, Init ESP32 Camera, and Setup MQTT
///
void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  wifiConnection();

  CameraWebServerAP.CameraWebServer_AP_Init();

  Serial.println("Elegoo-2020...");

  /* MQTT */
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  

  /* Web Socket */
  	// server address, port and URL
	webSocket.begin("192.168.15.54", 8080, "/");

	// event handler
	webSocket.onEvent(webSocketEvent);

	// use HTTP Basic Authorization this is optional remove if not needed
	webSocket.setAuthorization("user", "Password");

	// try ever 5000 again if connection has failed
	webSocket.setReconnectInterval(5000);
}

///
/// 
///
void loop()
{
  String sendBuff;

  // Serial to UNO
  /* MQTT */
  if(!client.connected()){
    reconnect();
  }
  client.loop();
  
  /* WEBSOCKET */
  webSocket.loop();


  captureStillImage();

  Serial2.print(messageMQTT);
  
  //sendBuff 
  if (Serial2.available())
      {
        sendBuff = Serial2.read();
        sendBuff = "";
      }      
}
