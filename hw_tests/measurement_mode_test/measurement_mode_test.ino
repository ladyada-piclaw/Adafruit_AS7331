#include <Adafruit_AS7331.h>
#include <Wire.h>

Adafruit_AS7331 as7331;

const as7331_mode_t modes[] = {AS7331_MODE_CONT, AS7331_MODE_CMD,
                               AS7331_MODE_SYNS, AS7331_MODE_SYND};
const char *mode_names[] = {"CONT", "CMD", "SYNS", "SYND"};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("AS7331 measurement mode test");
  if (!as7331.begin()) {
    Serial.println("FAIL: begin()");
    return;
  }

  bool overall_ok = true;

  for (uint8_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
    bool ok = true;
    if (!as7331.powerDown(true)) {
      Serial.println("FAIL: powerDown(true)");
      ok = false;
    }

    if (!as7331.setMeasurementMode(modes[i])) {
      Serial.print("FAIL: setMeasurementMode ");
      Serial.println(mode_names[i]);
      ok = false;
    }

    as7331_mode_t readback = as7331.getMeasurementMode();

    if (!as7331.powerDown(false)) {
      Serial.println("FAIL: powerDown(false)");
      ok = false;
    }

    if (readback != modes[i]) {
      Serial.print("FAIL: readback ");
      Serial.print(mode_names[i]);
      Serial.print(" got=");
      Serial.println(readback);
      ok = false;
    }

    if (ok) {
      Serial.print("PASS: ");
      Serial.println(mode_names[i]);
    } else {
      Serial.print("FAIL: ");
      Serial.println(mode_names[i]);
      overall_ok = false;
    }
  }

  if (overall_ok) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }
}

void loop() { delay(1000); }
