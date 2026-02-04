#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static const as7331_clock_t kClocks[] = {
  AS7331_CLOCK_1024MHZ,
  AS7331_CLOCK_2048MHZ,
  AS7331_CLOCK_4096MHZ,
  AS7331_CLOCK_8192MHZ,
};

static const char *kClockLabels[] = {
  "1024MHz",
  "2048MHz",
  "4096MHz",
  "8192MHz",
};

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
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.powerDown(false);

  bool overall_pass = true;

  for (uint8_t i = 0; i < (sizeof(kClocks) / sizeof(kClocks[0])); i++) {
    as7331.powerDown(true);
    as7331.setClockFrequency(kClocks[i]);
    as7331_clock_t readback = as7331.getClockFrequency();
    as7331.powerDown(false);

    delay(10);
    bool pass = (readback == kClocks[i]);
    overall_pass &= pass;

    Serial.print("Clock ");
    Serial.print(kClockLabels[i]);
    Serial.print(": readback=");
    Serial.print((int)readback);
    Serial.print(", ");
    Serial.println(pass ? "PASS" : "FAIL");
  }

  Serial.println(overall_pass ? "OVERALL PASS" : "OVERALL FAIL");
}

void loop() {
}
