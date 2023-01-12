#ifndef SMART_INTERCOM_MQTT_H
#define SMART_INTERCOM_MQTT_H

#include <PubSubClient.h>
#include <functional>

using Callback = std::function<void(const char *)>;

class Mqtt
{
private:
  PubSubClient client;
  const char *address;
  const char *username;
  const char *password;

public:
  Mqtt(const char *server, const char *user, const char *pass);
  void loop();
  bool connected();
  void connect();
  void subscribe(const char* topic, Callback callback);
  void publish(const char *topic, const char *value);
};

#endif
