#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static const uint8_t kTimes[] = {
    AS7331_TIME_1MS,  AS7331_TIME_2MS,  AS7331_TIME_4MS,  AS7331_TIME_8MS,
    AS7331_TIME_16MS, AS7331_TIME_32MS, AS7331_TIME_64MS,
};

static const uint16_t kTimeMs[] = {1, 2, 4, 8, 16, 32, 64};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  if (!as7331.begin()) {
    Serial.println("AS7331 not found");
    while (1) {
      delay(10);
    }
  }

  pixels.begin();
  pixels.setBrightness(255);
  for (uint16_t i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
  }
  pixels.show();

  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_4X);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.powerDown(false);

  bool overall_pass = true;
  uint16_t previous_uva = 0;

  for (uint8_t i = 0; i < sizeof(kTimes); i++) {
    as7331.powerDown(true);
    as7331.setIntegrationTime(kTimes[i]);
    as7331.powerDown(false);

    delay(kTimeMs[i] + 100);

    uint16_t uva = as7331.readUVA();

    if (i == 0) {
      Serial.print("Time ");
      Serial.print(kTimeMs[i]);
      Serial.print("ms: UVA=");
      Serial.println(uva);
    } else {
      uint32_t expected = (uint32_t)previous_uva * 2;
      uint32_t lower = (expected * 90) / 100;
      uint32_t upper = (expected * 110) / 100;
      bool pass = (uva >= lower) && (uva <= upper);
      overall_pass &= pass;

      Serial.print("Time ");
      Serial.print(kTimeMs[i]);
      Serial.print("ms: UVA=");
      Serial.print(uva);
      Serial.print(", expected=");
      Serial.print(expected);
      Serial.print(", ");
      Serial.println(pass ? "PASS" : "FAIL");
    }

    previous_uva = uva;

    if (uva > 60000) {
      Serial.println("UVA near saturation, stopping test");
      break;
    }
  }

  Serial.println(overall_pass ? "OVERALL PASS" : "OVERALL FAIL");
}

void loop() {}
