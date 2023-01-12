#include "mqtt.h"
#include <unordered_map>
#include <WiFi.h>

#define MQTT_TOPIC_PREFIX "smart_intercom/"

WiFiClient wifiClient;

// incoming message handling
std::unordered_map<std::string, Callback> callbacks;
void onMessage(const char *topic, byte *payload, unsigned length)
{
  char value[256];
  strncpy(value, (char *)payload, length);
  value[length] = 0; // null-terminated string

  Serial.print(topic);
  Serial.print(":");
  Serial.println(value);

  if (callbacks.count(topic))
    callbacks[topic](value);
}

Mqtt::Mqtt(const char *server, const char *user, const char *pass)
    : address(server), username(user), password(pass)
{
  client.setClient(wifiClient);
  client.setServer(server, 1883);
  client.setCallback(onMessage);
}

void Mqtt::loop()
{
  client.loop();
}

bool Mqtt::connected()
{
  return client.connected();
}

void Mqtt::connect()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", username, password))
    {
      Serial.print("Connected to ");
      Serial.println(address);

      for (auto &[topic, callback] : callbacks)
        client.subscribe(topic.c_str());
    }
    else
    {
      Serial.print("Connection failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void Mqtt::subscribe(const char *topic, Callback callback)
{
  auto fullTopic = std::string(MQTT_TOPIC_PREFIX) + topic;
  client.subscribe(fullTopic.c_str());
  callbacks[fullTopic] = callback;
}

void Mqtt::publish(const char *topic, const char *value)
{
  auto fullTopic = std::string(MQTT_TOPIC_PREFIX) + topic;
  client.publish(fullTopic.c_str(), value);
}
