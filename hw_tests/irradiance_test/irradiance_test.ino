#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("AS7331 Irradiance Test");

  pixels.begin();
  pixels.fill(pixels.Color(255, 255, 255));
  pixels.show();

  if (!as7331.begin()) {
    Serial.println("AS7331 not found!");
    while (1)
      delay(10);
  }

  // Test with LEDs ON at different gains
  Serial.println("\n--- UV LEDs ON ---");

  // Test at GAIN_4X
  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_4X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.powerDown(false);
  delay(200);

  float uva_4x = as7331.readUVA_uWcm2();
  float uvb_4x = as7331.readUVB_uWcm2();
  float uvc_4x = as7331.readUVC_uWcm2();
  Serial.print("GAIN_4X: UVA=");
  Serial.print(uva_4x, 2);
  Serial.print(" UVB=");
  Serial.print(uvb_4x, 2);
  Serial.print(" UVC=");
  Serial.print(uvc_4x, 2);
  Serial.println(" µW/cm²");

  // Test at GAIN_16X
  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_16X);
  as7331.powerDown(false);
  delay(200);

  float uva_16x = as7331.readUVA_uWcm2();
  Serial.print("GAIN_16X: UVA=");
  Serial.print(uva_16x, 2);
  Serial.println(" µW/cm²");

  // Check if readings are within 25% of each other (same light source)
  float ratio = (uva_4x > 0.001) ? (uva_16x / uva_4x) : 0;
  bool gain_match = (ratio > 0.75 && ratio < 1.25);
  Serial.print("Gain consistency (4x vs 16x): ratio=");
  Serial.print(ratio, 2);
  Serial.println(gain_match ? " PASS" : " FAIL");

  // Test with LEDs OFF
  pixels.clear();
  pixels.show();
  delay(200);

  Serial.println("\n--- UV LEDs OFF ---");
  float uva_off = as7331.readUVA_uWcm2();
  Serial.print("UVA with LEDs off: ");
  Serial.print(uva_off, 2);
  Serial.println(" µW/cm²");

  bool off_low = (uva_off < uva_4x * 0.1); // Should be much lower
  Serial.println(off_low ? "LEDs OFF test: PASS" : "LEDs OFF test: FAIL");

  Serial.println(gain_match && off_low ? "\nOverall: PASS" : "\nOverall: FAIL");
}

void loop() { delay(1000); }
