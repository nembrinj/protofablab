#include <Adafruit_NeoPixel.h>

#define LED_PIN 6 // which PIN of the arduino board is used to send 'commands' to the ring
#define LED_COUNT 24 // the numbe rof led on the ring

Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  ring.begin();
  ring.show();
  ring.setBrightness(50);
}


/*
* Example to make the led ring alterate Red, Green, Yellow colors and shift the position of these colors by 1  every 500 ms
* It gives the impression that the RGY colors move along the ring.
*/
int k = 0;
void loop() {
  for (int i = 0; i < LED_COUNT; i++) {
    if((i+k)%3 == 0){
      ring.setPixelColor(i, ring.Color(255, 255, 0));//yellow
    }else if((i+k)%2==0){
      ring.setPixelColor(i, ring.Color(255, 0, 0)); //Red
    }else{
      ring.setPixelColor(i, ring.Color(0, 255, 0)); //Green
    }
    delay(5);
  }
  ring.show()
  delay(500);
  k++;
}