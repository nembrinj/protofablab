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

const char* MQTT_STATUS_TOPIC = "order/status"; // Gives the status of completion of the order
const char* MQTT_WAITER_TOPIC = "waiter/status"; // Gives the status of the robot
const char* MQTT_DRINK_TOPIC = "waiter/order/drink"; // Gives the value of the drink that is ordered

// ### 3. HARDWARE SETTINGS ###

const int POWER_LED_PIN = 9; // Is turned on when Arduino is running.

const int BUTTON_PIN = 6;
unsigned long DEBOUNCE_DELAY = 50;

const int DRINK_LED_PINS[] = {10, 11, 12};
const int NB_DRINK_PINS = 3;

// ############ GLOBAL VARIABLES ############

bool is_busy = false; // True if TIAGo is serving an order

int debounced_button_state = LOW;
int last_button_state = LOW;
unsigned long last_debounce_time = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// ############ HELPER FUNCTIONS ############

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.println("Connecting to MQTT broker...");
        if (client.connect("tiagoClient")) {
            Serial.println("Connected to MQTT broker.");
            client.subscribe(MQTT_DRINK_TOPIC);
        } else {
            Serial.println("Failed to connect to MQTT broker, rc = " + String(client.state()) + ".");
            Serial.println("Trying again in 5 seconds.");
            delay(5000);
        }
    }
}

void onMessageReceived(char* topic, byte* payload, unsigned int length) {
    Serial.println("Message arrived [" + String(topic) + "]");

    // Print message
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    Serial.println(message);

    // Manage message
    if (strcmp(topic, MQTT_DRINK_TOPIC) == 0) {
      if (!is_busy && validateDrink(message)) {
        is_busy = true;
        turnOnDrinkLED(message);
        client.publish(MQTT_WAITER_TOPIC, "busy");
        client.publish(MQTT_STATUS_TOPIC, "accepted");
        Serial.println("Order accepted and waiter is now busy.");
      }
    } else {
      Serial.println("Error : Unrecognized topic.");
    }
}

bool validateDrink(String drink) {
  if (drink == "drink1" || drink == "drink2" || drink == "drink3") {
    return true;
  } else {
    Serial.println("Error : Unrecognized drink value.");
    return false;
  }
}

void turnOnDrinkLED(String drink) {
  for (int i = 0; i < NB_DRINK_PINS; i++) {
    digitalWrite(DRINK_LED_PINS[i], LOW);
  }
  if (drink == "drink1") {
    digitalWrite(DRINK_LED_PINS[0], HIGH);
  } else if (drink == "drink2") {
    digitalWrite(DRINK_LED_PINS[1], HIGH);
  } else if (drink == "drink3") {
    digitalWrite(DRINK_LED_PINS[2], HIGH);
  }
}

// ############ MAIN ############

void setup() {
    // Set up pins
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < NB_DRINK_PINS; i++) {
      pinMode(DRINK_LED_PINS[i], OUTPUT);
    }
    pinMode(POWER_LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);

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
    client.setCallback(onMessageReceived);

    digitalWrite(POWER_LED_PIN, HIGH); // Turn power LED on to show that the setup is done
}

void loop() {
    if (!client.connected()) {
      reconnectMQTT();
      if (!is_busy) {
          Serial.println("Waiter is free.");
          client.publish(MQTT_WAITER_TOPIC, "free");
      }
    }

    if (is_busy) {
      int current_button_state = digitalRead(BUTTON_PIN);

      if (current_button_state != last_button_state) {
        last_debounce_time = millis();
      }

      // If button is pressed long enough not to be considered a false positive
      if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
        if (current_button_state != debounced_button_state) {
          debounced_button_state = current_button_state;

          if (debounced_button_state == HIGH) {
            client.publish(MQTT_STATUS_TOPIC, "drink_received");
            Serial.println("Can on button");

            // Turn on the LEDs to show that the drink is on the TIAGo
            for (int i = 0; i < NB_DRINK_PINS; i++) {
              digitalWrite(DRINK_LED_PINS[i], HIGH);
            }
          } else {
            client.publish(MQTT_STATUS_TOPIC, "drink_picked_up");
            Serial.println("Can off button");
            client.publish(MQTT_WAITER_TOPIC, "free");
            Serial.println("Order is served and waiter is now free.");
            is_busy = false; // TIAGo finished serving the order

            // Turn off the LEDs to show that the drink is off the TIAGo
            for (int i = 0; i < NB_DRINK_PINS; i++) {
              digitalWrite(DRINK_LED_PINS[i], LOW);
            }
          }
        }
      }
      last_button_state = current_button_state;
    }
    client.loop();
}

