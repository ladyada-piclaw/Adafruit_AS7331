#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct BreakStep {
  uint8_t value;
  const char *label;
};

const BreakStep break_steps[] = {
    {0, "0 (0us)"},
    {25, "25 (200us)"},
    {125, "125 (1ms)"},
    {255, "255 (2.04ms)"},
};

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

  for (size_t i = 0; i < sizeof(break_steps) / sizeof(break_steps[0]); i++) {
    uint8_t value = break_steps[i].value;
    as7331.powerDown(true);
    bool wrote = as7331.setBreakTime(value);
    uint8_t readback = as7331.getBreakTime();
    as7331.powerDown(false);
    delay(10);
    bool pass = wrote && (readback == value);
    if (!pass) {
      all_pass = false;
    }

    Serial.print("Break time ");
    Serial.print(break_steps[i].label);
    Serial.print(": wrote=");
    Serial.print(value);
    Serial.print(", read=");
    Serial.print(readback);
    Serial.print(", ");
    Serial.println(pass ? "PASS" : "FAIL");
  }

  Serial.println(all_pass ? "Overall: PASS" : "Overall: FAIL");
}

void loop(void) {}
