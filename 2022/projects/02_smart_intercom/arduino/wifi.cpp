#include "wifi.h"
#include <WiFi.h>

void wifiConnect(const char *ssid, const char *password)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
    Serial.print(".");

  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
}
