# UNO: Smart Robot Car

This part is about the Arduino UNO R3 board which has an ELEGOO SmartCar-Shield (v1.1), a lithium-battery, four motors and wheels, and is connected to the ESP32.

## Installation

Elegoo kit using the Arduino IDE. A .ino file needs to be in a folder with the same name.


## Code explanation


In the main file [SmartRobotCarV4.0_V1_20220303.ino](./SmartRobotCarV4.0_V1_20220303/SmartRobotCarV4.0_V1_20220303.ino), our main contribution is called via the ```TeddyCtrlUARTMessage(String name)``` function which handles the UART message and forward to TeddyCTrlManager
```C++
Application_FunctionSet.TeddyCtrlUARTMessage("Zoro");
```

In more details, this function:

```C++
//
// Deserialize UART Message received from Serial
//
void ApplicationFunctionSet::TeddyCtrlUARTMessage(String name)
{
  // Data to pass to TeddyCtrlManager
  int xPosition = 0;
  int depth = 0;

  if (Serial.available() > 0) // (c == '$') //Data frame tail check
  {
    // Prepare filter
    StaticJsonDocument<192> doc;                               //Declare a JsonDocument object, in Stack (static)
    StaticJsonDocument<192> filter;

    JsonObject filter_teddyCtrl = filter.createNestedObject("teddyCtrl");

    JsonObject filter_teddyCtrl_0 = filter_teddyCtrl.createNestedObject("0");
    filter_teddyCtrl_0["xPos"] = true;
    filter_teddyCtrl_0["depth"] = true;
    filter_teddyCtrl_0["name"] = true;

    JsonObject filter_teddyCtrl_1 = filter_teddyCtrl.createNestedObject("1");
    filter_teddyCtrl_1["xPos"] = true;
    filter_teddyCtrl_1["depth"] = true;
    filter_teddyCtrl_1["name"] = true;
    
    // Deserialize and error handling
    DeserializationError error = deserializeJson(doc, Serial, DeserializationOption::Filter(filter)); //Deserialize JSON data from the serial data buffer
    if (error)
    {
      Serial.print("error:deserializeJson: ");
      Serial.println(error.c_str());
    }
    else if (!error) //Check if the deserialization is successful
    {
      for (JsonPair teddyCtrl_item : doc["teddyCtrl"].as<JsonObject>()) {
        const char* teddyCtrl_item_key = teddyCtrl_item.key().c_str(); // "0", "1"

        xPosition = teddyCtrl_item.value()["xPos"]; // 115, -18
        depth = teddyCtrl_item.value()["depth"]; // 48, 38
        const char* teddyCtrl_item_value_name = teddyCtrl_item.value()["name"]; // "unknown", "unknown"
        Serial.print(teddyCtrl_item_key);
        Serial.print(": xPos=");
        Serial.print(xPosition);
        Serial.print(", depth=");
        Serial.print(depth);
        Serial.print(", name=");
        Serial.println(teddyCtrl_item_value_name);
        if(strcmp(teddyCtrl_item_value_name, name.c_str()) == 0){
          TeddyCtrlManager(xPosition, depth);
          break;
        }
      }
    }
  }
}
```

```C++
//
// Update motors and servo based on deserialized data received from ESP
//
void ApplicationFunctionSet::TeddyCtrlManager(int xPosition, int depth)
{
  int alpha = Aiming(xPosition, depth);
  int orientation = TurnServo(alpha);
  MotorHandling(xPosition, depth, orientation, speed);

  if(depth > 20){
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, speed);
  
  }
}

```


## References

Information on the Elegoo kit:
 - [ELEGOO Kit's web page (v3)](https://www.elegoo.com/products/elegoo-smart-robot-car-kit-v-3-0-plus)
 - [Elegoo's google drive official code](https://drive.google.com/file/d/1OVjEPiXy-WVvtv_hjAU9QrVyp5uzDlWp/view)
 - [Amazon's link and info on Elegoo Kit (v4, actually used in our project)](https://www.amazon.com/ELEGOO-Tracking-Ultrasonic-Intelligent-Educational/dp/B07KPZ8RSZ)
