# ESP32 WROVER

For the ESP32, we build on top of the code and libraries provided by the robot kit from ELEGOO, with the [official code version here](https://drive.google.com/file/d/1OVjEPiXy-WVvtv_hjAU9QrVyp5uzDlWp/view). In addition, we used an example from this course for the wifi connection and MQTT, and one example from [WebSockets by Markus Sattler](https://github.com/Links2004/arduinoWebSockets/blob/master/examples/esp32/WebSocketClient/WebSocketClient.ino) for the websocket handling.

The other files from Elegoo were modified only to translate comments to English. For development, we used the Arduino IDE version 2.0.3, and downgraded the board manager's esp32 by Espressif Systems to version 1.0.6 to match the specificities of the official code from Elegoo.

## Explanation on ESP32_CameraServer_AP_20220120.ino

The main file modified is [ESP32_CameraServer_AP_20220120.ino](./ESP32_CameraServer_AP_20220120/ESP32_CameraServer_AP_20220120).ino]. We kept the initialization of the camera which is included in the ESP32 WROVER board. We added the WiFi, MQTT and WebSocket functionalities.

In the setup() function, the ESP32 initializes two serials, one for itself and a second one which will be used for UART communication with the Arduino UNO board. Then, using wifiConnection(), ESP32 establishes a WiFi ocnnection as a station to a mobile access point (created by ourselves with our Google Pixel 4a phone). Then, using CameraWebServerAP.CameraWebServer_AP_Init(), the camera is initialized. Afterwards, both MQTT and WebSocket are set up. 

In the loop() function, both MQTT and WebSocket have their on .loop() running. Then, we send each frame buffer from the camera in binary format in the captureStillImage() function. Finally, the MQTT message, received with the callback() and set with the setter Set_MessageMQTT(), is printed to the second serial for the Arduino to read via UART.

Signatures of functions:
```C++
// WiFi
void wifiConnection();

// MQTT related
void callback(char* topic, byte* payload, unsigned int length);
void Set_MessageMQTT(String message);
void reconnect();

// Image
void captureStillImage();

// WebSocket
void hexdump(const void *mem, uint32_t len, uint8_t cols = 16);
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);

// Arduino's default setup() and loop() functions:
void setup();
void loop();
```

## Installation

After downloading the official code from the [official link](https://drive.google.com/file/d/1OVjEPiXy-WVvtv_hjAU9QrVyp5uzDlWp/view), a couple of things need to be adjusted in the Arduino IDE.

In the Arduino IDE's library manager, install:

 - ArduinoJson by Benoit Blanchon, version 6.19.4
 - WebSockets by Markus Sattler version 2.3.6
 - PubSubClient by Nick O'Leary <nick.oleary@gmail.com> version 2.8
 - Protothreads by Ben Artin <ben@artings.org>, Adam Dunkels, verison 1.4.0-arduino.beta.1

 Menu Sketch > ... > add library zip:
 - Load library from ELEGOO
   - FastLED.h: C:\Users\cedri\Downloads\LL\kit20221026\ELEGOO Smart Robot Car Kit V4.0 2022.10.26\02 Manual & Main Code & APP\02 Main Program   (Arduino UNO)\TB6612 & MPU6050\SmartRobotCarV4.0_V1_20220303\addLibrary

 Important: make sure that you have 
 - selected Board ---> ESP32 Dev Module
 - Partition Scheme ---> Huge APP (3MB No OTA/1MB SPIFFS)
 - PSRAM ---> enabled