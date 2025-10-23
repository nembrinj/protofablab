#include <Arduino.h>
#include <FastLED.h>

#define PIN_POT 4
#define PIN_PWM 10
#define PIN_BTN 0
#define PIN_PIX 1

#define DEBOUNCE 0

#define NUM_PIX 1

#define LED_FREQ 200
#define LED_RESOLUTION 14

long ad_val;
float intensity;
float last_intensity;
long last_message_millis;

int address = 0;
int long_address = 0;
bool connected = false;
bool muted = false;

bool blink = false;

//----------------------------------------------------------------------------------------

CRGB pixel[NUM_PIX];

//========================================================================================
//----------------------------------------------------------------------------------------
//																				SETUP

void setup()
{
  FastLED.addLeds<SK6812, PIN_PIX, GRB>(pixel, NUM_PIX);
  FastLED.setBrightness(16);
  pixel[0] = CRGB::Yellow;
  FastLED.show();

  Serial.begin(115200);
  delay(150);

  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, LOW);

  ledcAttach(PIN_PWM, LED_FREQ, LED_RESOLUTION);

  log_v("_______________________");
  log_v("Setup DONE");
}

//========================================================================================
//----------------------------------------------------------------------------------------
//																				loop

void loop()
{
  static long last_leds;
  
  ad_val = (7 * ad_val + analogRead(PIN_POT)) / 8;

  intensity = (float)ad_val / 4095.;

  if (blink)
    intensity = 0;

  float f = pow(intensity, 4) * pow(2, LED_RESOLUTION);
  ledcWrite(PIN_PWM, floor(f));

  delay(2);

  if (millis()%100 > 50)
    blink = false;
  else 
    blink = true;

}
