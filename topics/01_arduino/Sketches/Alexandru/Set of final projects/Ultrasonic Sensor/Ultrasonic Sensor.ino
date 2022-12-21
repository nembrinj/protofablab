#include <WiFi.h>
#include <ThingESP.h>

#define echoPin 5
#define trigPin 6
                                    
ThingESP32 thing("Alexandru98", "Bathtub", "123456789");

int LED = LED_BUILTIN;
unsigned long previousMillis = 0;
const long INTERVAL = 6000;

long duration;
int distance;
int distanceUntilWater;
bool detectHighWater = true;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED, OUTPUT);

  Serial.begin(9600);

  thing.SetWiFi("redmi", "alexandru");
  thing.initDevice();
  distanceUntilWater = 30;

}

void loop() {
  
  // We clean the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // We set the trigPin to HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2


  if ( (distance > distanceUntilWater) && detectHighWater) {
    Serial.print(distance);
    Serial.println(" cm");
    detectHighWater = false;
    thing.sendMsg("+40770130157", String(distance));
  }
  
  thing.Handle();
}

String HandleResponse(String query){
  query.toLowerCase();


  if (query == "reset") {
    detectHighWater = true;
    return "Done: Looking for the height of the water again";
  }


  if (query == "led on") {
    digitalWrite(LED, 1);
    return "Done: LED Turned ON";
  }

  else if (query == "led off") {
    digitalWrite(LED, 0);
    return "Done: LED Turned OFF";
  }

  else if (query == "led status")
    return digitalRead(LED) ? "LED is ON" : "LED is OFF";


  else return "The query was invalid...";

}







