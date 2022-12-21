#include <LiquidCrystal.h>

LiquidCrystal lcd(11, 12, 10, 39, 38, 9);

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  lcd.print("Saludos amigos");
  lcd.setCursor(0,1);
  lcd.print("Como estais");
}

void loop() {
  // put your main code here, to run repeatedly:

}
