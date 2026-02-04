#include <Adafruit_AS7331.h>

Adafruit_AS7331 as7331;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("AS7331 Measurement Mode Test");

  if (!as7331.begin()) {
    Serial.println("AS7331 not found!");
    while (1) delay(10);
  }

  bool all_pass = true;
  const char* mode_names[] = {"CONT", "CMD", "SYNS", "SYND"};

  for (int i = 0; i < 4; i++) {
    as7331.powerDown(true);
    as7331.setMeasurementMode((as7331_mode_t)i);
    as7331_mode_t readback = as7331.getMeasurementMode();
    as7331.powerDown(false);

    bool pass = (readback == i);
    Serial.print("Mode ");
    Serial.print(mode_names[i]);
    Serial.print(": set=");
    Serial.print(i);
    Serial.print(", read=");
    Serial.print(readback);
    Serial.println(pass ? " PASS" : " FAIL");

    if (!pass) all_pass = false;
  }

  Serial.println(all_pass ? "Overall: PASS" : "Overall: FAIL");
}

void loop() {
  delay(1000);
}
