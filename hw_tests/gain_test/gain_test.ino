#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

struct GainStep {
  uint8_t gain;
  const char *label;
};

const GainStep gain_steps[] = {
    {AS7331_GAIN_1X, "1"}, {AS7331_GAIN_2X, "2"},   {AS7331_GAIN_4X, "4"},
    {AS7331_GAIN_8X, "8"}, {AS7331_GAIN_16X, "16"}, {AS7331_GAIN_32X, "32"},
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
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.powerDown(false);

  bool all_pass = true;

  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_1X);
  as7331.powerDown(false);
  delay(200);

  uint16_t previous_uva = as7331.readUVA();
  Serial.print("Gain 1x: UVA=");
  Serial.print(previous_uva);
  Serial.println(" (baseline)");

  for (size_t i = 1; i < sizeof(gain_steps) / sizeof(gain_steps[0]); i++) {
    as7331.powerDown(true);
    as7331.setGain(gain_steps[i].gain);
    as7331.powerDown(false);
    delay(200);

    uint16_t uva = as7331.readUVA();
    float expected = previous_uva * 2.0f;
    bool pass = (uva >= expected * 0.9f) && (uva <= expected * 1.1f);
    if (!pass) {
      all_pass = false;
    }

    Serial.print("Gain ");
    Serial.print(gain_steps[i].label);
    Serial.print("x: UVA=");
    Serial.print(uva);
    Serial.print(", expected=");
    Serial.print((uint16_t)expected);
    Serial.print(", ");
    Serial.println(pass ? "PASS" : "FAIL");

    if (uva > 60000) {
      Serial.println("Stopping: approaching saturation");
      break;
    }

    previous_uva = uva;
  }

  Serial.println(all_pass ? "Overall: PASS" : "Overall: FAIL");
}

void loop(void) {}
