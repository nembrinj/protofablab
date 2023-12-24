#ifndef MQTT_CLIENT
#define MQTT_CLIENT

#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <stdlib.h>
#include <cstring>
#include <mutex>

WiFiClient wlan_client;
PubSubClient pub_sub_client(wlan_client);

std::mutex mqtt_mutex;

struct DoorInformation {
  enum LockedUnlockedStatus {
    LOCKED_UNLOCKED_STATUS_UNKNOWN = 0,
    LOCKED = 1,
    UNLOCKED = 2
  };

  LockedUnlockedStatus locked_unlocked_status;
  // Milliseconds relative to when the ESP32 was last rebooted.
  unsigned long locked_unlocked_last_updated;

  enum OpenClosedStatus {
    OPEN_CLOSED_STATUS_UNKNOWN = 0,
    OPEN = 1,
    CLOSED = 2
  };

  OpenClosedStatus open_closed_status;
  // Milliseconds relative to when the ESP32 was last rebooted.
  unsigned long open_closed_last_updated;
};
DoorInformation door_information;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MqttCallback(const char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (std::strcmp(topic, kMqttDoorLockedUnlockedTopic) != 0
      && std::strcmp(topic, kMqttDoorOpenClosedTopic) != 0) {
    // Not interesting.
    return;
  }

  size_t pos = std::strcspn((char*)payload, " ");
  char* subtopic =
    (char*)(malloc((pos + 1) * sizeof(char)));
  std::strncpy(subtopic, (char*)payload, pos);
  subtopic[pos] = '\0';

  pos = std::strcspn((char*)payload, "=");
  char* status_as_string =
    (char*)(std::malloc((length - pos) * sizeof(char)));
  std::strncpy(status_as_string, ((char*)payload) + pos + 1, length - pos - 1);
  status_as_string[length - pos - 1] = '\0';

  if (std::strcmp(subtopic, kMqttLockedUnlocked) != 0
      && std::strcmp(subtopic, kMqttOpenClosed) != 0) {
    // Not interesting.
    return;
  }

  {
    std::lock_guard<std::mutex> lock_quard_mqtt_mutex(mqtt_mutex);
    unsigned long now = millis();
    if (std::strcmp(topic, kMqttDoorLockedUnlockedTopic) == 0 && std::strcmp(subtopic, kMqttLockedUnlocked) == 0) {
      DoorInformation::LockedUnlockedStatus status =
        (std::strcmp(status_as_string, "1") == 0 ? DoorInformation::LOCKED : (std::strcmp(status_as_string, "2") == 0 ? DoorInformation::UNLOCKED : DoorInformation::LOCKED_UNLOCKED_STATUS_UNKNOWN));
      door_information.locked_unlocked_status = status;
      door_information.locked_unlocked_last_updated = now;
    } else {
      DoorInformation::OpenClosedStatus status =
        (std::strcmp(status_as_string, "1") == 0 ? DoorInformation::OPEN : (std::strcmp(status_as_string, "2") == 0 ? DoorInformation::CLOSED : DoorInformation::OPEN_CLOSED_STATUS_UNKNOWN));
      door_information.open_closed_status = status;
      door_information.open_closed_last_updated = now;
    }
  }
}

void SubscribeDoorEvents() {
  Serial.println("MQTT::SubscribeDoorEvents");

  if (!pub_sub_client.subscribe(kMqttDoorLockedUnlockedTopic)) {
    Serial.print("MQTT::SubscribeDoorEvent: Failed to Subscribe to ");
    Serial.print(kMqttDoorLockedUnlockedTopic);
    Serial.println(".");
    exit(-1);
  }

  if (!pub_sub_client.subscribe(kMqttDoorOpenClosedTopic)) {
    Serial.print("MQTT::SubscribeDoorEvent: Failed to Subscribe to ");
    Serial.print(kMqttDoorOpenClosedTopic);
    Serial.println(".");
    exit(-1);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PublishVL53L4CXMeasurement(const int measurement) {
  Serial.println("MQTT::PublishVL53L4CXMeasurement");

  const String payload = "VL53L4CX measurement=" + String(measurement);
  if (!pub_sub_client.publish(kMqttVL53L4CXTopic, payload.c_str())) {
    Serial.println("MQTT::PublishVL53L4CXMeasurement: Failed to Publish.");
  }
}

void PublishAPDS9960Measurement(const int event) {
  Serial.println("MQTT::PublishAPDS9960Measurement");

  // Event can be 1=ENTER or -1=LEAVE
  const String payload = "APDS9960 event=" + String(event);
  if (!pub_sub_client.publish(kMqttAPDS9960Topic, payload.c_str())) {
    Serial.println("MQTT::PublishAPDS9960Measurement: Failed to Publish.");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup

void SetupMqtt() {
  Serial.println("MQTT::Setup");

  WiFi.mode(WIFI_STA);
  WiFi.begin(kExistingWifiName, kExistingWifiPassword);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(".");
  }

  pub_sub_client.setServer(kMqttServer, kMqttPort);
  pub_sub_client.setCallback(MqttCallback);

  while (!pub_sub_client.connected()) {
    Serial.println("MQTT::Setup: Connecting...");

    // if (!pub_sub_client.connect("ESP32", kMqttUser, kMqttPassword)) {
    if (!pub_sub_client.connect("ESP32")) {
      Serial.print("MQTT::Setup. Failed to connect: ");
      Serial.println(pub_sub_client.state());
      delay(1000);
    }
  }

  {
    // By default, status is unknown.
    std::lock_guard<std::mutex> lock_quard_mqtt_mutex(mqtt_mutex);
    unsigned long now = millis();
    door_information.locked_unlocked_status =
      DoorInformation::LOCKED_UNLOCKED_STATUS_UNKNOWN;
    door_information.locked_unlocked_last_updated = now;
    door_information.open_closed_status =
      DoorInformation::OPEN_CLOSED_STATUS_UNKNOWN;
    door_information.open_closed_last_updated = now;
  }

  SubscribeDoorEvents();

  Serial.println("MQTT::Setup: Connected.");
}

#endif  // MQTT_CLIENT