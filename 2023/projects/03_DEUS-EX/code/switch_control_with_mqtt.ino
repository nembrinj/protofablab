#include <WiFi.h>
#include <PubSubClient.h>
#include <ezButton.h>

ezButton limitSwitch(17);  // create ezButton object that attach to ESP32 pin GPIO17


const char* ssid = "";
const char* password = "";
WiFiClient wifiClient;

const char* mqttBroker = "";
const char* mqttClientName = "";
const String mqttUser = ""; // MQTT User Authentification
const String mqttPass = ""; // MQTT Password Authentification
const char* topicToSub = "flip_lock_state";
const char* topicToPub = "get_current_lock_state";
PubSubClient mqttClient(wifiClient);
const String receivedMsg;  //MQTT Message to lock/unlock


const int lockPin = 12;
const int unlockPin = 13;

void mqttConnect() {
  
  while (!mqttClient.connected()) {
  
    Serial.print("Attempting MQTT connection...");
  
    if (mqttClient.connect(mqttClientName, mqttUser, mqttPass)) {
  
      Serial.println("connected");
      mqttClient.publish("hello", mqttClientName);
      
      // Topic(s) subscription
      mqttClient.subscribe(topicToSub);
  
    } else {
      
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {

  for (int i = 0; i < length; i++) {
     receivedMsg += (char)message[i];
  }
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(receivedMsg);
}

void setup() {
  
  Serial.begin(115200);

  // Connect to wifi
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // MQTT setup
  mqttClient.setServer(mqttBroker, 1883);
  mqttClient.setCallback(callback);

  // Led setup
  pinMode(lockPin, OUTPUT);
  pinMode(unlockPin,OUTPUT);
}

char msg[50];

void loop() {
    
  limitSwitch.loop();
  int state = limitSwitch.getState(); //Getting the state of the switch

  // Check if we are still connected to the MQTT broker
  if (!mqttClient.connected()) {
    mqttConnect();
  }

  mqttClient.loop();

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