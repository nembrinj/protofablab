 #include <LiquidCrystal_I2C.h>

// LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.

void setup() {
  // Initiate the LCD:
  lcd.init();
  lcd.backlight();
}

void loop() {
  // Print 'Hello World!' on the first line of the LCD:
  lcd.setCursor(0, 0); // Set the cursor on the first column and first row.
  lcd.print("Hello Sandro!"); // Print the string "Hello World!"

  lcd.setCursor(0, 1); //Set the cursor on the first column and the second row (counting starts at 0!).
  lcd.print("This Works!");
}











