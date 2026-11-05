#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "WIFI";
const char* password = "PASSWORD";

WiFiUDP udp;
const unsigned int localPort = 4210;

void setup() {
    Serial.begin(115200);

    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    udp.begin(localPort);

    Serial.println("UDP server ready on port 4210");
}

void loop() {
    int packetSize = udp.parsePacket();

    if (packetSize > 0) {
        char buffer[64];

        int len = udp.read(buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            Serial.print("Received: ");
            Serial.println(buffer);
        }
    }
}