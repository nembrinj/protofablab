/*
 * ESP32 S0009 servo motor
 */
#include <WiFi.h>
#include <PubSubClient.h>

#include <ESP32Servo.h>

// create servo object to control a servo
Servo myservo;

// GPIO the servo is attached to
static const int servoPin = 13;

// Replace with your network credentials
const char *ssid = "...";
const char *password = "...";

// Put your MQTT broker credentials here
const char *mqttServer = "...";
const int mqttPort = 1883;
// const char* mqttUser = "mqttuser";
// const char* mqttPassword = "password";
const char *mqttTopic = "pen_state";

WiFiClient wlanClient;
PubSubClient client(wlanClient);

void setup(void)
{
  /* Serial Monitor setup */
  Serial.begin(115200);
  Serial.println("Wifi S0009 servo motor controller");

  // /* Servo setup */
  myservo.attach(servoPin); // attaches the servo on pin 13 to the servo object
  myservo.write(90);        // set servo initially to UP

  /* Wifi + server functions */
  WiFi.mode(WIFI_STA); // Connect to wifi
  WiFi.begin(ssid, password);

  Serial.println("Connecting to ");
  Serial.print(ssid);

  // Wait for WiFi to connect
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  // If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); // IP address assigned to your ESP

  /* MQTT setup */
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  connectToMqtt();
}

// Handle incoming MQTT messages
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");

  String message = "";
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Control servo motor based on pen state
  if (message == "up")
  {
    // Code to move the servo up
    myservo.write(90);
    Serial.println("Moving servo UP");
  }
  else if (message == "down")
  {
    // Code to move the servo down
    myservo.write(180);
    Serial.println("Moving servo DOWN");
  }
}

void connectToMqtt()
{
  while (!client.connected())
  {
    Serial.println("Connecting to MQTT...");

    // if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
    if (client.connect("ESP32Client"))
    {

      Serial.println("connected");

      if (client.subscribe(mqttTopic))
      {
        Serial.println("Successfully subscribed to topic");
        Serial.println(mqttTopic);
      }
      else
      {
        Serial.println("Failed to subscribe to topic");
      }
    }
    else
    {

      Serial.print("Failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void loop(void)
{
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi Disconnected. Attempting to reconnect...");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(1000);
      Serial.println("Connecting to WiFi...");
      WiFi.begin(ssid, password);
    }
    Serial.println("Connected to WiFi");
  }

  // Check MQTT connection
  if (!client.connected())
  {
    Serial.println("MQTT disconnected. Attempting to reconnect...");
    connectToMqtt();
  }
  // Process incoming messages and maintain MQTT connection
  client.loop();
}