#include <Adafruit_AS7331.h>

Adafruit_AS7331 as7331;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  if (as7331.begin()) {
    Serial.println("AS7331 found!");
  } else {
    Serial.println("AS7331 not found");
  }
}

void loop() {
  delay(1000);
}
