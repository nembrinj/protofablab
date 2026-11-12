
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <math.h>

// ==========================================
// 1. WI-FI CONFIGURATION
// ==========================================

const char* ssid = "WIFI";
const char* password = "PASSWORD";

WiFiUDP udp;
const int localPort = 4210;


// ==========================================
// 2. SERVO CONFIGURATION
// ==========================================

Servo servo1;

const int servoPin = 6;

float pos = 70.0;
float vel = 0.0;

const float SERVO_MIN_ANGLE = 45.0;
const float SERVO_MAX_ANGLE = 100.0;

// Pulse width configuration
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;


// ==========================================
// 3. SAFETY CONFIGURATION
// ==========================================

const float MAX_VELOCITY = 8.0;

const unsigned long COMMAND_TIMEOUT_MS = 500;

unsigned long lastCommandTime = 0;
unsigned long lastUpdateTime = 0;

float lastPrintedVelocity = 999.0;


// ==========================================
// 4. SERVO POSITION CONTROL
// ==========================================

void updateServo() {

    int pulseWidth = map(
        (int)(pos * 100),
        0,
        18000,
        SERVO_MIN_US,
        SERVO_MAX_US
    );

    servo1.writeMicroseconds(pulseWidth);
}


// ==========================================
// 5. SETUP
// ==========================================

void setup() {

    Serial.begin(115200);

    // Initialize servo
    servo1.setPeriodHertz(50);

    servo1.attach(
        servoPin,
        SERVO_MIN_US,
        SERVO_MAX_US
    );

    updateServo();

    Serial.println("Servo initialized.");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);

    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("Wi-Fi connected!");

    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());

    // Start UDP
    udp.begin(localPort);

    Serial.print("Listening on UDP port: ");
    Serial.println(localPort);

    lastCommandTime = millis();
    lastUpdateTime = millis();
}


// ==========================================
// 6. MAIN LOOP
// ==========================================

void loop() {

    unsigned long now = millis();

    // --------------------------------------
    // A. RECEIVE UDP COMMANDS
    // --------------------------------------

    int packetSize = udp.parsePacket();

    if (packetSize > 0) {

        char buffer[32];

        int len = udp.read(
            buffer,
            sizeof(buffer) - 1
        );

        if (len > 0) {

            buffer[len] = '\0';

            char* endPtr;

            float newVelocity = strtof(
                buffer,
                &endPtr
            );

            // Validate numeric command
            if (
                endPtr != buffer &&
                *endPtr == '\0' &&
                isfinite(newVelocity)
            ) {

                vel = constrain(
                    newVelocity,
                    -MAX_VELOCITY,
                    MAX_VELOCITY
                );

                lastCommandTime = now;

                // Print only when velocity changes
                if (fabs(vel - lastPrintedVelocity) >= 0.01) {

                    Serial.print("New velocity: ");
                    Serial.println(vel);

                    lastPrintedVelocity = vel;
                }
            }
        }
    }


    // --------------------------------------
    // B. WATCHDOG
    // --------------------------------------

    if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {

        vel = 0.0;
    }


    // --------------------------------------
    // C. WIFI SAFETY
    // --------------------------------------

    if (WiFi.status() != WL_CONNECTED) {

        vel = 0.0;
    }


    // --------------------------------------
    // D. UPDATE SERVO POSITION
    // --------------------------------------

    float dt = (now - lastUpdateTime) / 1000.0;

    lastUpdateTime = now;

    // Avoid large jumps after delays
    dt = min(dt, 0.05f);

    pos += vel * dt;

    // Apply mechanical limits
    pos = constrain(
        pos,
        SERVO_MIN_ANGLE,
        SERVO_MAX_ANGLE
    );

    // Update servo pulse
    updateServo();

    delay(10);
}
