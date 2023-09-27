#include <WiFi.h>
#include <ThingESP.h>

ThingESP32 thing("Alexandru98", "Bathtub", "123456789");

int LED = LED_BUILTIN;

unsigned long previousMillis = 0;
const long INTERVAL = 6000;  

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  thing.SetWiFi("redmi", "alexandru");

  thing.initDevice();

}


String HandleResponse(String query)
{

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


  else return "The query was invalid..";

}


void loop()
{
  // millis() Returns the number of milliseconds passed since the Arduino board began running the current program. 
  if (millis() - previousMillis >= INTERVAL) {  
    previousMillis = millis();

    String msg = digitalRead(LED) ? "LED is ON" : "LED is OFF";
    
    thing.sendMsg("+40770130157", msg);
 
  }

  

    thing.Handle(); 

}