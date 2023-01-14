#include "wifi.h"
#include "mqtt.h"

const char *WIFI_SSID = "ssid";
const char *WIFI_PASSWORD = "password";

const char *MQTT_SERVER = "address";
const char *MQTT_USER = "user";
const char *MQTT_PASSWORD = "pass";

unsigned sampleRate = 4; // in Hz

double offset = 0;   // the DC offset
unsigned window = 1; // amount of samples to average

double maxAmp = 0;      // the highest amplitude recorded
double dropRate = 1000; // how much maxAmp decreases every second

double minBellAmp = 50;       // above this = ringing
double minBellDuration = 500; // longer than this = ringing
double bellDuration = 0;      // the current bell duration
bool ringing = false;         // true if currently ringing

Mqtt mqtt(MQTT_SERVER, MQTT_USER, MQTT_PASSWORD);

void setup()
{
  Serial.begin(115200);

  pinMode(A1, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  wifiConnect(WIFI_SSID, WIFI_PASSWORD);
  mqtt.connect();

  mqtt.subscribe("bell_amp", [](const char *val) { minBellAmp = atof(val); });
  mqtt.subscribe("bell_duration", [](const char *val) { minBellDuration = atof(val); });
  mqtt.subscribe("sample_rate", [](const char *val) { sampleRate = max(atoi(val), 1); });
  mqtt.subscribe("drop_rate", [](const char *val) { dropRate = atof(val); });
  mqtt.subscribe("door_action", [](const char *val) {
    if (!strcmp(val, "open")) {
      // here we would activate the servo motor
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("opening door");
      delay(5000); 
      digitalWrite(LED_BUILTIN, LOW);
  }});
}

unsigned counter = 0;

void loop()
{
  ++counter;

  if (!mqtt.connected())
    mqtt.connect();

  mqtt.loop();

  // read amplitude
  unsigned amp = analogRead(A1);

  // update max, or drecrease slowly
  if (amp > maxAmp - dropRate / sampleRate)
    maxAmp = amp;
  else
    maxAmp -= dropRate / sampleRate;

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
    mqtt.publish("bell_ringing", "true");
  }
  else if (ringing && bellDuration < minBellDuration)
  {
    ringing = false;
    mqtt.publish("bell_ringing", "false");
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
