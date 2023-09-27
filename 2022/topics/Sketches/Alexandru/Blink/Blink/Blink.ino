
const int PIN_NUMBER = 5;

void setup() {
  pinMode(PIN_NUMBER, OUTPUT);
}

void loop() {
  digitalWrite(PIN_NUMBER, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(PIN_NUMBER, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
