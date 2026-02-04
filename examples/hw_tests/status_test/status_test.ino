#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  pixels.begin();
  pixels.setBrightness(255);
  pixels.fill(pixels.Color(255, 255, 255));
  pixels.show();

  if (!as7331.begin()) {
    Serial.println("AS7331 not found");
    while (1) {
      delay(10);
    }
  }

  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_4X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.powerDown(false);

  bool all_pass = true;

  delay(200);
  uint8_t status = as7331.getStatus();
  Serial.print("Status raw: 0x");
  if (status < 0x10) {
    Serial.print("0");
  }
  Serial.println(status, HEX);

  delay(100);
  bool new_data = as7331.hasNewData();
  Serial.print("Test 2 (hasNewData after wait): ");
  Serial.println(new_data ? "PASS" : "FAIL");
  if (!new_data) {
    all_pass = false;
  }

  bool no_overflow = !as7331.hasOverflow();
  Serial.print("Test 3 (no overflow at normal light): ");
  Serial.println(no_overflow ? "PASS" : "FAIL");
  if (!no_overflow) {
    all_pass = false;
  }

  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_2048X);
  as7331.setIntegrationTime(AS7331_TIME_2048MS);
  as7331.powerDown(false);
  delay(2200);

  bool overflow = as7331.hasOverflow();
  Serial.print("Test 4 (overflow at max gain/long integration): ");
  Serial.println(overflow ? "PASS" : "FAIL");

  Serial.println(all_pass ? "Overall: PASS" : "Overall: FAIL");
}

void loop(void) {
}
