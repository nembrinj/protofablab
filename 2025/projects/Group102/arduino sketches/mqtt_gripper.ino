#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

const char* ssid        = "ProFab";
const char* password    = "1700_UniFR.&";
const char* mqtt_server = "192.168.1.17";   // broker IP

WiFiClient espClient;
PubSubClient client(espClient);
Servo claw;

const int SERVO_PIN    = 6;   // <-- CHANGE THIS (do NOT use 6)
const int OPEN_ANGLE   = 50;
const int CLOSE_ANGLE  = 160;
const int MOVE_DELAYMS = 600;

void setup_wifi() {
  Serial.print("WiFi connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();  // IMPORTANT

  Serial.print("[MQTT] "); Serial.print(topic); Serial.print(" = "); Serial.println(msg);

  if (String(topic) == "claw/cmd") {
    if (msg == "CLOSE") {
      claw.write(CLOSE_ANGLE);
      delay(MOVE_DELAYMS);
      client.publish("claw/state", "CLOSED");
    } else if (msg == "OPEN") {
      claw.write(OPEN_ANGLE);
      delay(MOVE_DELAYMS);
      client.publish("claw/state", "OPENED");
    }
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    if (client.connect("ESP32_CLAW_CLIENT")) {
      Serial.println("connected");
      client.subscribe("claw/cmd");
      Serial.println("subscribed claw/cmd");
    } else {
      Serial.print("failed rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Better servo stability on ESP32
  claw.setPeriodHertz(50);
  claw.attach(SERVO_PIN, 500, 2400);
  claw.write(OPEN_ANGLE);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();
}
