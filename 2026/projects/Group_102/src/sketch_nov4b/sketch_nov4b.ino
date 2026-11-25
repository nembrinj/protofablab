#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <math.h>

// Moves the camera servo to the angle received over UDP (e.g. "72.50").
// The angle, its limits and the start angle are decided in ROS
// (servo_tilt_node.py); this sketch only moves the servo.

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

// Pulse width for 0 and 180 degrees.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

float lastPrintedAngle = -1.0;


// ==========================================
// 3. SETUP
// ==========================================

void setup() {

    Serial.begin(115200);

    // The servo is attached on the first angle from ROS, so it does not
    // move anywhere before ROS says where.
    servo1.setPeriodHertz(50);

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
}


// ==========================================
// 4. MAIN LOOP
// ==========================================

void loop() {

    int packetSize = udp.parsePacket();

    if (packetSize > 0) {

        char buffer[32];

        int len = udp.read(buffer, sizeof(buffer) - 1);

        if (len > 0) {

            buffer[len] = '\0';

            char* endPtr;
            float angle = strtof(buffer, &endPtr);

            // Only accept a clean number.
            if (endPtr != buffer && *endPtr == '\0' && isfinite(angle)) {

                angle = constrain(angle, 0.0, 180.0);

                if (!servo1.attached()) {
                    servo1.attach(servoPin, SERVO_MIN_US, SERVO_MAX_US);
                }

                int pulseWidth = map((int)(angle * 100), 0, 18000, SERVO_MIN_US, SERVO_MAX_US);
                servo1.writeMicroseconds(pulseWidth);

                // Print only when the angle changes.
                if (fabs(angle - lastPrintedAngle) >= 0.5) {
                    Serial.print("New angle: ");
                    Serial.println(angle);
                    lastPrintedAngle = angle;
                }
            }
        }
    }

    delay(5);
}
