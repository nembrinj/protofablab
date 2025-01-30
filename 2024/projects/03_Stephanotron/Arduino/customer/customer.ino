#include <WiFi.h>
#include <PubSubClient.h>

// ############ CONSTANTS ############

// ### 1. WIFI SETTINGS ###

const char* SSID = "ProFab";
const char* PASSWORD = "1700_UniFR.&";

// ### 2. MQTT SETTINGS ###

const char* MQTT_SERVER = "raspcg.local"; // Set to Christine's pi
// const char* MQTT_SERVER = "raspberrypidrm.local"; // Set to Dario's pi

const int MQTT_PORT = 1883;

const char* MQTT_DRINK_TOPIC = "order/drink"; // Gives the value of the drink that is ordered

// ### 3. HARDWARE SETTINGS ###

const int POWER_LED_PIN = 10; // Is turned on when Arduino is running.

const int DRINK_BUTTON_PINS[] = {5, 6, 9};
const int NB_DRINK_BUTTONS = 3;
unsigned long DEBOUNCE_DELAY = 50;

// ############ GLOBAL VARIABLES ############

int debounced_button_states[] = {LOW, LOW, LOW};
int last_button_states[] = {LOW, LOW, LOW};
unsigned long last_debounce_times[] = {0, 0, 0};

WiFiClient espClient;
PubSubClient client(espClient);

// ############ HELPER FUNCTIONS ############

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.println("Connecting to MQTT broker...");
        if (client.connect("customerClient")) {
            Serial.println("Connected to MQTT broker.");
            client.subscribe(MQTT_DRINK_TOPIC);
        } else {
            Serial.println("Failed to connect to MQTT broker, rc = " + String(client.state()) + ".");
            // Serial.print(client.state());
            Serial.println("Trying again in 5 seconds.");
            delay(5000);
        }
    }
}

// ############ MAIN ############

void setup() {
    // Set up pins
    pinMode(POWER_LED_PIN, OUTPUT);
    for (int i = 0; i < NB_DRINK_BUTTONS; i++) {
      pinMode(DRINK_BUTTON_PINS[i], INPUT);
    }
    Serial.begin(115200);

    // Set up WIFI
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi.");

    // Set up MQTT
    client.setServer(MQTT_SERVER, MQTT_PORT);

    digitalWrite(POWER_LED_PIN, HIGH); // Turn power LED on to show that the setup is done
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }

  int current_button_states[NB_DRINK_BUTTONS];

  for (int i = 0; i < NB_DRINK_BUTTONS; i++) {
    current_button_states[i] = digitalRead(DRINK_BUTTON_PINS[i]);

    if (current_button_states[i] != last_button_states[i]) {
      last_debounce_times[i] = millis();
    }

    // If button is pressed long enough not to be considered a false positive
    if ((millis() - last_debounce_times[i]) > DEBOUNCE_DELAY) {
      if (current_button_states[i] != debounced_button_states[i]) {
        debounced_button_states[i] = current_button_states[i];

        // Button pressed == client is ordering something
        if (debounced_button_states[i] == HIGH) {
          String drink_name = "drink" + String(i+1);
          Serial.println("New order : " + drink_name);
          client.publish(MQTT_DRINK_TOPIC, drink_name.c_str());
        }
      }
    }
    last_button_states[i] = current_button_states[i];
  }

  client.loop();
}
