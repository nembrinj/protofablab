#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <FastLED.h>
#include "driver/ledc.h"

// Pines
#define PIN_POT 4
#define PIN_PWM 10
#define PIN_BTN 0
#define PIN_PIX 1

// LED setup
#define NUM_PIX 1
#define LED_FREQ 200
#define LED_RESOLUTION LEDC_TIMER_14_BIT
#define LED_TIMER LEDC_TIMER_0
#define LED_CHANNEL LEDC_CHANNEL_0

// WiFi y MQTT
const char* ssid = "WIFI";
const char* password = "password";
const char* mqtt_server = "IP broker MQTT";
const int mqtt_port = 1883;
const char* mqtt_topic = "light/intensity";

// Variables
float pot_intensity = 0;
float mqtt_intensity = -1;  // -1 means nothing has been received
float last_intensity = 0;

CRGB pixel[NUM_PIX];
WiFiClient espClient;
PubSubClient client(espClient);

// MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';  // end of string
  String msg = String((char*)payload);

  float val = msg.toFloat();
  if (val >= 0.0 && val <= 1.0) {
    mqtt_intensity = val;
    Serial.printf("MQTT -> New intensity: %.2f\n", mqtt_intensity);
  }
}

// WiFi and MQTT connection
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to broker MQTT...");
    if (client.connect("ESP32C3_LUZ")) {
      Serial.println("Connected");
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("Error, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 2s...");
      delay(2000);
    }
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  FastLED.addLeds<SK6812, PIN_PIX, GRB>(pixel, NUM_PIX);
  pixel[0] = CRGB::Blue;
  FastLED.setBrightness(16);
  FastLED.show();

  // Set up PWM
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LED_RESOLUTION,
    .timer_num = LED_TIMER,
    .freq_hz = LED_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num = PIN_PWM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LED_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LED_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);

  // Connect to WiFi
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected IP: %s\n", WiFi.localIP().toString().c_str());

  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("Setup DONE");
}

// LOOP
void loop() {
  // Maintain MQTT
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // Read from physical input
  int raw_value = analogRead(PIN_POT);
  pot_intensity = (float)raw_value / 4095.0;

  // Prioritize the MQTT value
  float target_intensity = (mqtt_intensity > 0.0) ? mqtt_intensity : pot_intensity;

  float f = pow(target_intensity, 2.2);
  uint32_t duty = (uint32_t)(f * pow(2, 14));

  if (fabs(duty - last_intensity) > 1) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LED_CHANNEL);
    last_intensity = duty;
  }

  delay(5);
}
