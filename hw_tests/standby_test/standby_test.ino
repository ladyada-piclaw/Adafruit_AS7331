#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool testStandby(const char *label, bool set_value, bool expected) {
  bool ok = as7331.setStandby(set_value);
  bool read_value = as7331.getStandby();
  bool pass = ok && (read_value == expected);

  Serial.print(label);
  Serial.print(": set=");
  Serial.print(set_value ? "true" : "false");
  Serial.print(" read=");
  Serial.print(read_value ? "true" : "false");
  Serial.print(" => ");
  Serial.println(pass ? "PASS" : "FAIL");
  return pass;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  pixels.begin();
  pixels.fill(pixels.Color(255, 255, 255));
  pixels.show();

  if (!as7331.begin()) {
    Serial.println("AS7331 begin failed");
    while (1) {
      delay(10);
    }
  }

  as7331.setGain(AS7331_GAIN_4X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);

  bool pass1 = testStandby("Test 1", false, false);
  bool pass2 = testStandby("Test 2", true, true);
  bool pass3 = testStandby("Test 3", false, false);

  bool overall = pass1 && pass2 && pass3;
  Serial.print("Overall: ");
  Serial.println(overall ? "PASS" : "FAIL");
}

void loop() { delay(1000); }
