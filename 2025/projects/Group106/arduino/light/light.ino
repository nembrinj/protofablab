#define POT_PIN 4
#define LED_PIN 10

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int potValue = analogRead(POT_PIN);
  int brightness = map(potValue, 0, 4095, 0, 255);
  analogWrite(LED_PIN, brightness);
  delay(10);
}
