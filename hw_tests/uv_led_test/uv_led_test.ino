#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NUM_LEDS 60

Adafruit_NeoPixel strip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("UV LED NeoPixel Test");
  Serial.println("Turning on all 30 UV LEDs...");

  strip.begin();
  strip.setBrightness(255); // Full brightness

  // Set all pixels to white (UV LEDs will emit UV)
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  }
  strip.show();

  Serial.println("UV LEDs ON - check for glow");
}

void loop() {
  // Nothing to do - LEDs stay on
  delay(1000);
}
