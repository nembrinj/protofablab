#include <Wire.h>
#include <VL53L0X.h>
#include <ThingESP.h>
#include <cmath>
#include <Preferences.h>

VL53L0X sensor;
#define LONG_RANGE
#define HIGH_ACCURACY
const String myNumber = "+40770130157";
const String infoText = "Welcome to the Home Notifier, you are connected to the \
  bathtub sensor, we have to configure the device in order to remember the different water levels, \
  write the following numbers here in the chat: '1', '2' or '3'. Other available commands are 'info' and 'reset'. ";

Preferences preferences;

ThingESP32 thing("Alexandru98", "Bathtub", "123456789");


struct LevelsTrigger{
  bool level_1;
  bool level_2;
  bool level_3;
};

struct Levels{
  int level_1;
  int level_2;
  int level_3;
};

Levels levels = {120, 100, 80};

LevelsTrigger levelsTrigger = {false, false, false};

bool firstTimeLaunch;

void setup() 
{  
    //////////////////////
  Serial.begin(9600);
  // For the Back-end
  thing.SetWiFi("redmi", "alexandru");
  thing.initDevice();
  //////
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
  Levels levels = initializeLevels();
  levelsTrigger = initializeLevelsTrigger();

  thing.sendMsg("+40770130157", infoText);
  int level1 = preferences.getInt("level1", 0);
  int level2 = preferences.getInt("level2", 0);
  int level3 = preferences.getInt("level3", 0);

  while(level1 != 0 && level2 != 0 && level3 != 0)
  {
    int level1 = preferences.getInt("level1", 0);
    int level2 = preferences.getInt("level2", 0);
    int level3 = preferences.getInt("level3", 0);
    Serial.print("The height in CM for Level 1 is: ");      
    Serial.print(preferences.getInt("level1", 0));
    Serial.println();

    Serial.print("The height in CM For Level 2 is: ");      
    Serial.print(preferences.getInt("level2", 0));
    Serial.println();

    Serial.print("The height in CM For Level 3 is: ");      
    Serial.print(preferences.getInt("level3", 0));
    Serial.println();

    thing.Handle();
    delay(2000);
  }
  Serial.print("All 3 levels are ready!!!!!"); 


  Serial.print("The height in CM for Level 1 is: ");      
  Serial.print(preferences.getInt("level1", 0));
  Serial.println();

  Serial.print("The height in CM For Level 2 is: ");      
  Serial.print(preferences.getInt("level2", 0));
  Serial.println();

  Serial.print("The height in CM For Level 3 is: ");      
  Serial.print(preferences.getInt("level3", 0));
  Serial.println();

  thing.Handle();

}

void loop(){
  int distance = sensor.readRangeSingleMillimeters();
  int distanceInCm = round(distance/10.0);
  if(distanceInCm <= 200){
  Serial.print(distanceInCm + "cm FROM THE LOOP");
  Serial.println();
  // writeToWhatsApp(ditanceInCm);
  int aux = checkLevel(distanceInCm);
  SendWhatsAppOrNot(aux, levelsTrigger);  
  }

  if (sensor.timeoutOccurred()) { 
      Serial.print(" TIMEOUT");
  }

  thing.Handle();
  delay(4000);
}


// void writeToWhatsApp(int distanceInCM){
//     if((distanceInCM <= levels.level_1) && !levelsTrigger.level_1){
//         thing.sendMsg(myNumber, "Level 1 reached!");
//         levelsTrigger.level_1 = true;
//     } else if((distanceInCM <= levels.level_2) && !levelsTrigger.level_2){
//         thing.sendMsg(myNumber, "Level 2 reached!");
//         levelsTrigger.level_2 = true;
//     } else if( (distanceInCM <= levels.level_3) && !levelsTrigger.level_3){
//         thing.sendMsg(myNumber, "Level 3 reached!");
//         levelsTrigger.level_3 = true;
//     }
//     thing.Handle();
// }

int checkLevel(int distanceInCm){
  int checkLevel;
  levels = initializeLevels();
  if(distanceInCm > levels.level_1){
        checkLevel = 0;
    } else if((levels.level_2 < distanceInCm) && (distanceInCm <= levels.level_1)){
        checkLevel = 1;
    } else if((levels.level_3 <= distanceInCm) && (distanceInCm < levels.level_2)){
        checkLevel = 2;
    } else if(levels.level_3 > distanceInCm){
        checkLevel = 3;
    }
  return checkLevel;
}

void SendWhatsAppOrNot(int checkLevel, LevelsTrigger levelsTrigger){
  if(checkLevel = 1 && levelsTrigger.level_1){
    thing.sendMsg(myNumber, "You are in level 1!");
    thing.sendMsg("+51976584130", "You are in level 1!");
    levelsTrigger.level_1 = false;
  } else if (checkLevel = 2 && levelsTrigger.level_2){
    thing.sendMsg(myNumber, "You are in level 2!");
    thing.sendMsg("+51976584130", "You are in level 2!");
    levelsTrigger.level_2 = false;
  } else if (checkLevel = 3 && levelsTrigger.level_3){
    thing.sendMsg(myNumber, "You are in level 3!");
    thing.sendMsg("+51976584130", "You are in level 3!");
    levelsTrigger.level_3 = false;
  }

}

Levels initializeLevels(){
  Levels levels; 
  levels.level_1 = preferences.getInt("level1", 0);
  levels.level_2 = preferences.getInt("level2", 0);
  levels.level_3 = preferences.getInt("level3", 0);
  return levels;
}

LevelsTrigger initializeLevelsTrigger(){
  LevelsTrigger levelsTrigger; 
  levelsTrigger.level_1 = preferences.getBool("level1", false);
  levelsTrigger.level_2 = preferences.getBool("level2", false);
  levelsTrigger.level_3 = preferences.getBool("level3", false);
  return levelsTrigger;
}


String HandleResponse(String query){
  query.toLowerCase();

  if(query == "info"){
    return infoText;
  }
  if(query == "1"){
    int distance = sensor.readRangeSingleMillimeters();
    int ditanceInCm = round(distance/10.0);
    Serial.println("Distance in CM FROM 1" + ditanceInCm);
    while(ditanceInCm >= 200){
      ditanceInCm = round(sensor.readRangeSingleMillimeters()/10.0);
    }
    Serial.print(ditanceInCm + "cm distance saved on memory for the level 1.");
    Serial.println();
    preferences.putInt("level1", ditanceInCm);
    preferences.putBool("level1", true);
    return String(preferences.getInt("level1", 0)); 
  }  

  if(query == "2"){
    int distance = sensor.readRangeSingleMillimeters();
    int ditanceInCm = round(distance/10.0);
    while(ditanceInCm >= 200){
      ditanceInCm = round(sensor.readRangeSingleMillimeters()/10.0);
    }
    Serial.print(ditanceInCm + "cm distance saved on memory for the level 2.");
    Serial.println();
    preferences.putInt("level2", ditanceInCm);
    preferences.putBool("level2", true);
    return String(preferences.getInt("level2", 0)); 
  }  

  if(query == "3"){
    int distance = sensor.readRangeSingleMillimeters();
    int ditanceInCm = round(distance/10.0);
    while(ditanceInCm >= 200){
      ditanceInCm = round(sensor.readRangeSingleMillimeters()/10.0);
    }
    Serial.print(ditanceInCm + "cm distance saved on memory for the level 3.");
    Serial.println();
    preferences.putInt("level3", ditanceInCm);
    preferences.putBool("level3", true);
    return String(preferences.getInt("level3", 0)); 
  }  

  if(query == "reset"){
    levelsTrigger.level_1 = true;
    levelsTrigger.level_2 = true;
    levelsTrigger.level_3 = true;
  }
 

  return "The query was invalid...";

}



