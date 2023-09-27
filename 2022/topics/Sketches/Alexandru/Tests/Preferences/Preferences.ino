#include <Preferences.h>

String words;

Preferences preferences;
void setup() {
   Serial.begin(9600);
   preferences.begin("app", false);

   words = preferences.getString("SSID", "Not Set Value");
   Serial.print("Hey!");
}

void loop() {

    if (Serial.available() > 0) 
    {
      words = Serial.readString(); 
      preferences.putString("SSID", words);
    }

    Serial.println(preferences.getString("SSID", "not Set Value"));

    delay(1000);  // delay in between reads for stability

}


