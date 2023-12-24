#ifndef CONFIGS
#define CONFIGS

// #define HOME_CONFIG
// #define ON_TABLE

// WIFI
#ifdef HOME_CONFIG
const char* kExistingWifiName = "TODO";
const char* kExistingWifiPassword = "TODO;
#else
const char* kExistingWifiName = "ProFab";
const char* kExistingWifiPassword = "1700_UniFR.&";
#endif  // HOME_CONFIG

// Server
const char* ssid = "amesp32";
const char* password = "qwerty12345pass";

// MQTT
#ifdef HOME_CONFIG
const char* kMqttServer = "ampi.home";
#else
const char* kMqttServer = "ampi.local";
#endif  // HOME_CONFIG
const int kMqttPort = 1883;
const char* kMqttUser = "ProFab_mqttuser";
const char* kMqttPassword = "ProFab_mqtt_pass";

const char* kMqttVL53L4CXTopic = "esp32/VL53L4CX";

const char* kMqttAPDS9960Topic = "esp32/APDS9960";

const char* kMqttDoorLockedUnlockedTopic = "door/event/locked_unlocked";
const char* kMqttDoorOpenClosedTopic = "door/event/open_closed";
const char* kMqttLockedUnlocked = "locked_unlocked";
const char* kMqttOpenClosed = "open_closed";

// Information about the surroundings, used to decide what measurements are worth publishing.
#ifdef HOME_CONFIG
#ifdef ON_TABLE
const int kMinimumDistanceObject = 10;
// Distance to lamp: ~670
const int kMaximumDistanceObject = 600;
#else // HOME_CONFIG, not ON_TABLE
const int kMinimumDistanceObject = 10;
// Distance to wall: 1000
const int kMaximumDistanceObject = 900;
#endif  // ON_TABLE
#else // not HOME_CONFIG

#ifdef ON_TABLE
// walk 450-650
const int kMinimumDistanceObject = 10;
// Distance to wall is ~1600, so it's safe to assume that everything above 1500 is not interesting.
const int kMaximumDistanceObject = 600;
#else // not ON_TABLE
// walk 450-650
const int kMinimumDistanceObject = 200;
// Distance to wall is ~1600, so it's safe to assume that everything above 1500 is not interesting.
const int kMaximumDistanceObject = 1500;
#endif  // ON_TABLE

#endif  // HOME_CONFIG

#endif  // CONFIGS
