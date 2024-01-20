#include <ezButton.h>
#include <WiFi.h>

int analogPin = A1;
const char* ssid = "";
const char* pass = "";
const int outputPin = 13;
const int outputPin2 = 12;


void setupWifi(){
  delay(100);
  Serial.println("Connecting to");
  Serial.print(ssid);

  WiFi.begin(ssid,pass);

 
  Serial.println("Connected to");
  Serial.print(ssid);
}


void setup() {
  Serial.begin(9600);
  limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
  pinMode(outputPin, OUTPUT);
  pinMode(outputPin2,OUTPUT);
  // setupWifi();

}

void loop() {
   // MUST call the loop() function first


  int state = limitSwitch.getState();
  if(state == HIGH){
    //Switch is untouched with metal

    Serial.println("The limit switch: UNTOUCHED");

    if (receivedMsg == ("lock")) {  //Received message to lock
     digitalWrite(outputPin2,HIGH);
    }

    if (receivedMsg == ("unlock")) {
    //Completing the unlock process & publishing to the topic
     digitalWrite(outputPin,LOW);
     snprintf (msg, 50, "unlocked");      
     mqttClient.publish(topicToPub, msg);    
     }
  }
  else{
    //Switch is touched with metal

    Serial.println("The limit switch: TOUCHED");

     if (receivedMsg == ("unlock")) { //Received message to unlock
        digitalWrite(outputPin,HIGH);
     }

    if (receivedMsg == ("lock")) {
     //Completing the lock process & publishing to the topic
     digitalWrite(outputPin2,LOW);
     snprintf (msg, 50, "locked");      
     mqttClient.publish(topicToPub, msg);
    }
  
  }
    
}