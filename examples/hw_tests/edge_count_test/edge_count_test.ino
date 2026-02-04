#include <Adafruit_AS7331.h>
#include <Wire.h>

Adafruit_AS7331 as7331;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("AS7331 edge count test");
  if (!as7331.begin()) {
    Serial.println("FAIL: begin()");
    return;
  }

  uint8_t values[] = {1, 10, 100, 255};
  bool ok = true;

  for (uint8_t i = 0; i < sizeof(values); i++) {
    uint8_t value = values[i];

    if (!as7331.powerDown(true)) {
      Serial.println("FAIL: powerDown(true)");
      ok = false;
      continue;
    }

    if (!as7331.setEdgeCount(value)) {
      Serial.print("FAIL: setEdgeCount ");
      Serial.println(value);
      ok = false;
    }

    uint8_t readback = as7331.getEdgeCount();

    if (!as7331.powerDown(false)) {
      Serial.println("FAIL: powerDown(false)");
      ok = false;
    }

    if (readback == value) {
      Serial.print("PASS: ");
      Serial.println(value);
    } else {
      Serial.print("FAIL: ");
      Serial.print(value);
      Serial.print(" readback=");
      Serial.println(readback);
      ok = false;
    }
  }

  if (ok) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }
}

void loop() {
  delay(1000);
}
