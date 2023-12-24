#include <Arduino.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <vl53l4cx_class.h>
#include <Wire.h>
#include <mutex>
#include <optional>

#include "configs.h"
#include "mqtt_client.h"

#define XSHUT_PIN A1

#define APDS9960_ENABLED

VL53L4CX vl53l4cx_sensor;
TwoWire *DEV_I2C = &Wire;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef APDS9960_ENABLED

#include "Adafruit_APDS9960.h"

Adafruit_APDS9960 apds9960_sensor;

void SetupAPDS9960() {
  Serial.println("APDS9960::Setup");

  if (!apds9960_sensor.begin()) {
    Serial.println("APDS9960::Setup: Failed to Initialize Sensor.");
  }

  // Gesture mode will be entered once proximity mode senses something close.
  apds9960_sensor.enableProximity(true);
  apds9960_sensor.enableGesture(true);

  Serial.println("APDS9960::Setup: Sensor initialized");
}

void ReadGesture() {
  const uint8_t gesture = apds9960_sensor.readGesture();
  switch (gesture) {
    case APDS9960_UP:
      {
        Serial.println("ENTER");
        PublishAPDS9960Measurement(1);
        break;
      }
    case APDS9960_DOWN:
      {
        Serial.println("LEAVE");
        PublishAPDS9960Measurement(-1);
        break;
      }
    default:
      break;
  }
}

#endif  // APDS9960_ENABLED
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool LogIfError(const String message, const VL53L4CX_Error error, const bool loop_indefinitely = false) {
  if (error == VL53L4CX_ERROR_NONE) {
    // Not an error.
    return false;
  }

  Serial.print(message);
  Serial.println(error);

  if (!loop_indefinitely) {
    // Programm should continue.
    return true;
  }

  while (true)
    ;
  // Never reached.
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// setup

bool I2CDeviceAvailable(uint8_t address, TwoWire **wire) {
  // Check if device is available at the expected address
  Wire.begin();
  Wire.beginTransmission(address);

  VL53L4CX_Error error = Wire.endTransmission();
  if (error) {
    Wire.end();
    return false;
  }

  *wire = &Wire;
  Wire.end();
  return true;
}

void SetupVL53L4CX() {
  Serial.println("VL53L4CX::Setup");

  delay(10000);

  if (!I2CDeviceAvailable(VL53L4CX_DEFAULT_DEVICE_ADDRESS >> 1, &DEV_I2C)) {
    Serial.println("VL53L4CX::Setup: Error initializing VL53L4CX Sensor.");
    while (true)
      ;
  }

  DEV_I2C->begin();
  vl53l4cx_sensor.setI2cDevice(DEV_I2C);
  vl53l4cx_sensor.setXShutPin(XSHUT_PIN);

  // Configure VL53L4CX satellite component.
  vl53l4cx_sensor.begin();

  // Switch off VL53L4CX satellite component.
  vl53l4cx_sensor.VL53L4CX_Off();

  //Initialize VL53L4CX satellite component.
  VL53L4CX_Error error = vl53l4cx_sensor.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  LogIfError(/*message=*/"VL53L4CX::Setup: Failed to Initialize Sensor: ", error, /*loop_indefinitely=*/true);
  Serial.println("VL53L4CX::Setup: Sensor initialized.");

  delay(500);

  // Set distance mode.
  error = vl53l4cx_sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  LogIfError(/*message=*/"VL53L4CX::Setup: Failed to set Distance Mode: ", error, /*loop_indefinitely=*/true);
  Serial.println("VL53L4CX::Setup: Distance mode set.");

  delay(500);

  // Start Measurement.
  error = vl53l4cx_sensor.VL53L4CX_StartMeasurement();
  LogIfError(/*message=*/"VL53L4CX::Setup: Failed to Start Measurement: ", error, /*loop_indefinitely=*/true);
  Serial.println("VL53L4CX::Setup: Start Measurement.");
}

void setup() {
  Serial.begin(115200);

  delay(2000);

  // Wait until serial port opens for native USB devices.
  while (!Serial) {
    delay(500);
  }

  pinMode(LED_BUILTIN, OUTPUT);

  SetupVL53L4CX();
  // SetupServer();
  SetupMqtt();

#ifdef APDS9960_ENABLED
  SetupAPDS9960();
#endif  // APDS9960_ENABLED
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// loop

void ReadAndPrintMeasurement() {
  // Serial.println("VL53L4CX::ReadAndPrintMeasurement");

  uint8_t measurement_data_ready = 0;
  VL53L4CX_Error error = vl53l4cx_sensor.VL53L4CX_GetMeasurementDataReady(&measurement_data_ready);
  if (LogIfError(/*message=*/"VL53L4CX::ReadAndPrintMeasurement: Failed to read Measurement Data: ", error)) {
    return;
  }
  if (!measurement_data_ready) {
    Serial.println("VL53L4CX::ReadAndPrintMeasurement: Measurement Data not ready.");
    return;
  }

  VL53L4CX_MultiRangingData_t multi_ranging_data;
  VL53L4CX_MultiRangingData_t *multi_ranging_data_pointer = &multi_ranging_data;
  error = vl53l4cx_sensor.VL53L4CX_GetMultiRangingData(multi_ranging_data_pointer);
  if (LogIfError(/*message=*/"VL53L4CX::ReadAndPrintMeasurement: Failed to get Multi Ranging Data: ", error)) {
    return;
  }

  const int no_of_object_found = multi_ranging_data_pointer->NumberOfObjectsFound;
  Serial.println("VL53L4CX::ReadAndPrintMeasurement: " + String(no_of_object_found) + " objects found.");

  // Get the closest object only.
  std::optional<int16_t> minimim_range_mm = std::nullopt;
  for (int object_index = 0; object_index < no_of_object_found; ++object_index) {
    if (multi_ranging_data_pointer->RangeData[object_index].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID || multi_ranging_data_pointer->RangeData[object_index].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE) {
      const int16_t range_mm = multi_ranging_data_pointer->RangeData[object_index].RangeMilliMeter;
      if (!minimim_range_mm.has_value()) {
        minimim_range_mm = range_mm;
      } else {
        minimim_range_mm = std::min(*minimim_range_mm, range_mm);
      }
    }
  }
  if (minimim_range_mm.has_value()) {
    for (int k = 0; k < *minimim_range_mm / 10; k++) {
      Serial.print(" ");
    }
    Serial.println(*minimim_range_mm);

    // Publish only if interesting
    const int measurement = *minimim_range_mm;
    if (measurement >= kMinimumDistanceObject && measurement <= kMaximumDistanceObject) {
      PublishVL53L4CXMeasurement(measurement);
    }
  }

  if (error == 0) {
    error = vl53l4cx_sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    LogIfError(/*message=*/"VL53L4CX::ReadAndPrintMeasurement: Failed to Clear Intrerrupt and Start Measurement: ", error);
  }
}

void loop() {
  Serial.println("==================================================");
  ReadAndPrintMeasurement();

#ifdef APDS9960_ENABLED
  ReadGesture();
#endif  // APDS9960_ENABLED

  server.handleClient();
  pub_sub_client.loop();

  {
    std::lock_guard<std::mutex> lock_quard_mqtt_mutex(mqtt_mutex);
    if (door_information.locked_unlocked_status == DoorInformation::LOCKED) {
      // If the door is locked, any measurement we get in this time is not important.
      delay(1000);
    } else {
      if (door_information.open_closed_status == DoorInformation::CLOSED) {
        // If the door is closed, any measurement we get in this time is not important.
        delay(100);
      }
      // If UNLOCKED/UNKNOWN and OPEN/UNKNOWN
      delay(50);
    }
  }
}
