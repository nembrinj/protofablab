#include <Wire.h>
#include <VL53L0X.h>
#include <ThingESP.h>
#include <cmath>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiManager.h>


VL53L0X sensor;
#define LONG_RANGE
#define HIGH_ACCURACY

unsigned long previousMillis = 0;
const long INTERVAL = 6000;

Preferences preferences;
String myNumber = " ";
ThingESP32 thing("Alexandru98", "Bathtub", "123456789");

int level1;
int level2;
int level3;

bool level1Set;
bool level2Set;
bool level3Set;

bool volatileLevel1IsNotSet = true;
bool volatileLevel2IsNotSet = true;
bool volatileLevel3IsNotSet = true;


void setup() {
  Serial.begin(9600);
  // WiFi Manager code!///////////////////////////
  WiFiManager wfm;

  wfm.setDebugOutput(false);
  wfm.resetSettings();
  
  WiFiManagerParameter my_phone_number("my_number", "Enter your phone number!", "default" , 15);
  wfm.addParameter(&my_phone_number);

  if(!wfm.autoConnect("HN Water Sensor", "password")){
    Serial.println("failed to connect and hit timeout");
    ESP.restart();
    delay(1000);
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); 

  Serial.print("Custom text box entry: ");
  Serial.println(my_phone_number.getValue());
  myNumber = my_phone_number.getValue();

  ///////////////////////////////////////////////  IF VALUE IS DEFAULT BE IN WHILE LOOP UNTIL ITS NOT 'DEFAULT'
  // thing.SetWiFi("redmi", "alexandru");
  thing.initDevice();
  Wire.begin();
  sensor.setTimeout(500);

  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  #if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_ACCURACY
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(300000);
  #endif

  preferences.begin("App", false);

  thing.sendMsg(myNumber, "Welcome to the bathtub!");

  areLevelsSet();
  while(verifyIfSet() == false){
    areLevelsSet();
    if (millis() - previousMillis >= INTERVAL){
      previousMillis = millis();
      thing.sendMsg(myNumber, "If you see this message, you will have to setup the bathtub sensor. \n Send 1 2 and 3 for the different levels that you want.");
    }

    thing.Handle();
  }
  Serial.println("We exited the WHILE inside the SETUP()!");
  thing.Handle();
}

bool verifyIfSet(){
    if(level1Set == false){
      return false;
    }
    if (level2Set == false){
      return false;
    }
    if (level3Set == false){
      return false;
    }
    return true;  
}

void loop() {
  // put your main code here, to run repeatedly:
  int level = checkLevel(getDistanceCM());  

  Serial.println();
  if (millis() - previousMillis >= INTERVAL){
    previousMillis = millis();
    maybeSendToWhatsapp(level);
  }

  thing.Handle();

}

int checkLevel(int detectedDistance){
  int foundLevel = 0;   
  setLevels();

  Serial.println("CheckLevel Detected distance: " + detectedDistance);
  Serial.println("Level1: " + String(level1));
  Serial.println("Level2: " + String(level2));
  Serial.println("Level3: " + String(level3));

  if( detectedDistance <= level1 && detectedDistance >= level2 ){
    foundLevel = 1;
  } else if( detectedDistance <= level2 && detectedDistance >= level3 ) {
    foundLevel = 2;    
  } else if (detectedDistance < level3){
    foundLevel = 3;
  }
  return foundLevel;
}

void maybeSendToWhatsapp(int receivedLevel){

  Serial.println();  
  if(receivedLevel == 1 && volatileLevel1IsNotSet){
    volatileLevel1IsNotSet = false;
    Serial.println("Send to Whatsapp about the  level1!");
    thing.sendMsg(myNumber, "Attention, Level 1 has been reached!"); 
  } else if (receivedLevel == 2 && volatileLevel2IsNotSet) {
    volatileLevel2IsNotSet = false;
    thing.sendMsg(myNumber, "Attention, Level 2 has been reached!");      
  } else if (receivedLevel == 3 && volatileLevel3IsNotSet){
    volatileLevel3IsNotSet = false;
    thing.sendMsg(myNumber, "Attention, Level 3 has been reached!");    
  }
    
}

void areLevelsSet(){
  level1Set = preferences.getBool("level1_bool");
  level2Set = preferences.getBool("level2_bool");
  level3Set = preferences.getBool("level3_bool");
}

void setLevels(){
   level1 = preferences.getInt("level1");
   level2 = preferences.getInt("level2");
   level3 = preferences.getInt("level3");
}


int getDistanceCM(){
  int distance = sensor.readRangeSingleMillimeters();
  int distanceInCM = round(distance/10.0);

  while(distanceInCM >= 200){
    distanceInCM = round(sensor.readRangeSingleMillimeters()/10.0);
  }
  Serial.println();
  Serial.println("DETECTED distance in CM:" + String(distanceInCM));
  return distanceInCM;
}


String HandleResponse(String query){
  query.toLowerCase();

  if(query == "info" || query == "status"){
    String concatenation;
    concatenation.concat("Level 1: ");
    concatenation.concat(level1);
    concatenation.concat(" Level 2: ");
    concatenation.concat(level2);
    concatenation.concat(" Level 3: ");
    concatenation.concat(level3);
    return concatenation;
  }

  if(query == "1") {
    level1 = getDistanceCM();
    preferences.putBool("level1_bool", true);
    preferences.putInt("level1", level1);
    return String(level1);
  }

  if(query == "2") {
    level2 = getDistanceCM();
    preferences.putBool("level2_bool", true);
    preferences.putInt("level2", level2);
    return String(level2);
  }

  if(query == "3") {
    level3 = getDistanceCM();
    preferences.putBool("level3_bool", true);
    preferences.putInt("level3", level3);
    return String(level3);
  }

  if (query == "set true"){
    preferences.putBool("level1_bool", true);
    preferences.putBool("level2_bool", true);
    preferences.putBool("level3_bool", true);
    return "All the levels were set to true";
  }

    if (query == "reset"){
      preferences.putBool("level1_bool", false);
      preferences.putBool("level2_bool", false);
      preferences.putBool("level3_bool", false);
    return "All the levels were set to false";
  }


  return "The query was invalid...";
}





















