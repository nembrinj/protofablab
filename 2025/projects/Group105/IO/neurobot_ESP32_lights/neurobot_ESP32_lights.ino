#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================
// --- USER CONFIGURATION (EDIT THIS!) ---
// ============================================================

const char* ssid = "ProFab";        
const char* password = "1700_UniFR.&";
const char* mqtt_server = "192.168.1.105";    // <--- EDIT THIS (Your Pi IP)

#define PIN_NEO_PIXEL  5
#define NUM_PIXELS     30 

// ============================================================

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

// Variables for State
char currentState = 'N'; 
unsigned long lastUpdate = 0;

// --- Animation Specific Variables ---
// Happy
int chaseIndex = 0;

// Stressed
bool strobeState = false;

// Calm (FadeInOut logic variables)
int calmK = 0;
bool calmGoingUp = true;

// Focused (RunningLights logic variables)
int focusedPosition = 0;

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  NeoPixel.begin();
  NeoPixel.setBrightness(200); 
  NeoPixel.show();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

// --- LOOP ---
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  runAnimation();
}

// --- MQTT CALLBACK ---
void callback(char* topic, byte* message, unsigned int length) {
  char command = (char)message[0];
  if (command == 'H' || command == 'C' || command == 'F' || command == 'S' || command == 'N') {
    currentState = command;
    // Reset strip on state change for clean transition
    if (currentState == 'N') {
      NeoPixel.clear();
      NeoPixel.show();
    }
  }
}

// --- ANIMATION CONTROLLER ---
void runAnimation() {
  switch (currentState) {
    case 'H': animHappy(); break;
    case 'C': animCalm(); break;
    case 'F': animFocused(); break;
    case 'S': animStressed(); break;
    default: break;
  }
}

// ============================================================
// --- ANIMATION PATTERNS ---
// ============================================================

// 1. HAPPY: Theater Chase (Green)
void animHappy() {
  if (millis() - lastUpdate > 100) { 
    lastUpdate = millis();
    NeoPixel.clear();
    for(int i=0; i<NUM_PIXELS; i++) {
      if ((i + chaseIndex) % 3 == 0) {
        NeoPixel.setPixelColor(i, NeoPixel.Color(0, 255, 0)); // Green
      }
    }
    NeoPixel.show();
    chaseIndex++;
  }
}

// 2. CALM: Fade In/Out (Blue)
// Adapted from your "FadeInOut" snippet to be non-blocking
void animCalm() {
  // Speed control: Updates every 5ms creates a smooth fade
  if (millis() - lastUpdate > 5) { 
    lastUpdate = millis();

    // Logic from your snippet: Calculate 'k' up to 255 then down
    float k = calmK;
    
    // Calculate Blue intensity based on k (k/256.0 * 255)
    // We add a floor of '10' to ensure the LEDs never turn fully off
    float b = (k/256.0) * 255.0;
    if (b < 10) b = 10; 

    // Set all pixels to Blue
    for(int i=0; i<NUM_PIXELS; i++ ) {
      NeoPixel.setPixelColor(i, NeoPixel.Color(0, 0, (int)b));
    }
    NeoPixel.show();

    // Increment/Decrement logic
    if (calmGoingUp) {
      calmK++;
      if (calmK >= 255) calmGoingUp = false;
    } else {
      calmK -= 2; // Your snippet decreased by 2 (faster fade out)
      if (calmK <= 0) calmGoingUp = true;
    }
  }
}

// Global variable for breathing
uint8_t breathBrightness = 50;
int8_t breathDirection = 1;

// 3. FOCUSED: Simple purple breathing
void animFocused() {
  if (millis() - lastUpdate > 30) {
    lastUpdate = millis();
    
    // Update breathing brightness
    breathBrightness += breathDirection * 2;
    if (breathBrightness >= 150) breathDirection = -1;
    if (breathBrightness <= 20) breathDirection = 1;
    
    // Set ALL pixels to the same purple with wave position offset
    focusedPosition++;
    
    for (int i = 0; i < NUM_PIXELS; i++) {
      // Create spatial wave
      float spatialWave = (sin((i * 0.5) + (focusedPosition * 0.05)) + 1.0) / 2.0;
      uint8_t localBright = (uint8_t)(spatialWave * breathBrightness);
      
      // CRITICAL: Blue must be higher than red for visible purple!
      // Try ratio of 1:2 or 1:3 (red:blue)
      uint8_t r = localBright / 2;      // Half brightness for red
      uint8_t b = localBright;          // Full brightness for blue
      
      NeoPixel.setPixelColor(i, NeoPixel.Color(r, 0, b));
    }
    NeoPixel.show();
  }
}


// 4. STRESSED: Fast Strobe (Red)
void animStressed() {
  if (millis() - lastUpdate > 80) { 
    lastUpdate = millis();
    if (strobeState) NeoPixel.fill(NeoPixel.Color(0, 0, 0));
    else NeoPixel.fill(NeoPixel.Color(255, 0, 0));
    strobeState = !strobeState;
    NeoPixel.show();
  }
}

// ============================================================
// --- WI-FI FUNCTIONS ---
// ============================================================

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // Flash Blue waiting for Wifi
    NeoPixel.setPixelColor(0, NeoPixel.Color(0,0,50)); 
    NeoPixel.show();
    delay(50);
    NeoPixel.setPixelColor(0, NeoPixel.Color(0,0,0));
    NeoPixel.show();
  }
  Serial.println("WiFi connected");
  // Flash Green for Success
  NeoPixel.fill(NeoPixel.Color(0,50,0));
  NeoPixel.show();
  delay(500);
  NeoPixel.clear();
  NeoPixel.show();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe("/neurobot/lights/set");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}