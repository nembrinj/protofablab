#include <WiFi.h>
#include <PubSubClient.h>

const char *WIFI_SSID = "loris-mobile";
const char *WIFI_PASSWORD = "agrafe-cheval";

const char *MQTT_SERVER = "192.168.236.102";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "api";
const char *MQTT_PASSWORD = "apipw";
#define MQTT_TOPIC "smart_intercom"

unsigned sampleRate = 1000; // in Hz

double offset = 0;   // the DC offset
unsigned window = 1; // amount of samples to average

double maxAmp = 0;             // the highest amplitude recorded
const double DROP_RATE = 1000; // how much maxAmp decreases every second

double minBellAmp = 50;       // above this = ringing
double minBellDuration = 500; // longer than this = ringing
double bellDuration = 0;
bool ringing = false;

WiFiClient wlanClient;
PubSubClient mqtt(wlanClient);

char* toStr(double value)
{
  char buffer[256];
  dtostrf(value, 0, 2, buffer);
  return buffer;
}

char* toStr(unsigned value)
{
  char buffer[256];
  itoa(value, buffer, 10);
  return buffer;
}

void setup()
{
  Serial.begin(115200);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to WiFi...");

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
    Serial.print(".");

  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  while (!mqtt.connected())
  {
    Serial.println("Connecting to MQTT...");
    if (mqtt.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD))
    {
      Serial.print("Connected to ");
      Serial.println(MQTT_SERVER);
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(mqtt.state());
      delay(2000);
    }
  }

  mqtt.subscribe(MQTT_TOPIC "/bell_amp");
  mqtt.subscribe(MQTT_TOPIC "/bell_duration");
  mqtt.subscribe(MQTT_TOPIC "/sample_rate");
  mqtt.subscribe(MQTT_TOPIC "/door_action");
}

unsigned counter = 0;

void loop()
{
  ++counter;
  mqtt.loop();

  // read amplitude
  unsigned amp = analogRead(A1);

  // update max, or drecrease slowly
  if (amp > maxAmp)
    maxAmp = amp;
  else
    maxAmp -= DROP_RATE / sampleRate;

  // estimate DC offset by averaging amplitude during the last second
  offset = (offset * (window - 1) + amp) / window;
  if (window < sampleRate)
    ++window;

  // the amplitude that we will actually test
  double detectedAmp = maxAmp - offset;

  // measure how long the bell is ringing
  if (detectedAmp >= minBellAmp)
    bellDuration += 1000.0 / sampleRate;
  else
    bellDuration = 0;

  // publish message if bell starts or stops ringing
  if (!ringing && bellDuration >= minBellDuration)
  {
    ringing = true;
    mqtt.publish(MQTT_TOPIC "/bell_ringing", "true");
  }
  else if (ringing && bellDuration < minBellDuration)
  {
    ringing = false;
    mqtt.publish(MQTT_TOPIC "/bell_ringing", "false");
  }

  if ((counter * 10) % sampleRate == 0)
  {
    Serial.print("MinBellAmp:");
    Serial.print(minBellAmp);
    Serial.print(",MinBellDuration:");
    Serial.print(minBellDuration);
    Serial.print(amp);
    Serial.print(",Amplitude:");
    Serial.print(amp);
    Serial.print(",DC Offset:");
    Serial.print(offset);
    Serial.print(",Max:");
    Serial.print(maxAmp);
    Serial.print(",Detected:");
    Serial.print(detectedAmp);
    Serial.print(",Bell:");
    Serial.println(ringing ? 1000 : 0);
  }

  delayMicroseconds(1000000 / sampleRate);
}

void mqttCallback(const char *topic, byte *payload, unsigned length)
{
  char value[256];
  strncpy(value, (char *)payload, length);
  value[length] = 0;

  Serial.print(topic);
  Serial.print(":");
  Serial.println(value);

  if (!strcmp(topic, MQTT_TOPIC "/bell_amp"))
    minBellAmp = atof(value);

  else if (!strcmp(topic, MQTT_TOPIC "/bell_duration"))
    minBellDuration = atof(value);

  else if (!strcmp(topic, MQTT_TOPIC "/sample_rate"))
    sampleRate = min(atoi(value), 1);

  else if (!strcmp(topic, MQTT_TOPIC "/door_action") && !strcmp(value, "open"))
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("opening door");
    delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
}
