#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct DividerStep {
  uint8_t div;
  const char *label;
  float expected_factor;
};

const DividerStep divider_steps[] = {
    {0, "2", 0.5f},
    {1, "4", 0.25f},
    {2, "8", 0.125f},
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
  as7331.enableDivider(false);
  as7331.powerDown(false);
  delay(200);

  uint16_t baseline = as7331.readUVA();
  Serial.print("Divider disabled: UVA=");
  Serial.print(baseline);
  Serial.println(" (baseline)");

  bool all_pass = true;

  for (size_t i = 0; i < sizeof(divider_steps) / sizeof(divider_steps[0]);
       i++) {
    as7331.powerDown(true);
    as7331.enableDivider(true);
    as7331.setDivider(divider_steps[i].div);
    as7331.powerDown(false);
    delay(200);

    uint16_t uva = as7331.readUVA();
    float expected = baseline * divider_steps[i].expected_factor;
    bool pass = (uva >= expected * 0.85f) && (uva <= expected * 1.15f);
    if (!pass) {
      all_pass = false;
    }

    Serial.print("Divider / ");
    Serial.print(divider_steps[i].label);
    Serial.print(": UVA=");
    Serial.print(uva);
    Serial.print(", expected=");
    Serial.print((uint16_t)expected);
    Serial.print(", ");
    Serial.println(pass ? "PASS" : "FAIL");
  }

  Serial.println(all_pass ? "Overall: PASS" : "Overall: FAIL");
}

void loop(void) {}
