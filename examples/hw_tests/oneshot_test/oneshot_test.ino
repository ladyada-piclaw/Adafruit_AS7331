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

  Serial.println("AS7331 oneShot Test");

  pixels.begin();
  pixels.fill(pixels.Color(255, 255, 255));
  pixels.show();

  if (!as7331.begin()) {
    Serial.println("AS7331 not found!");
    while (1)
      delay(10);
  }

  // Configure gain/time before oneShot
  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_4X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.powerDown(false);

  bool all_pass = true;

  // Test oneShot with raw counts
  Serial.println("\nTesting oneShot() raw counts:");
  uint16_t uva, uvb, uvc;
  if (as7331.oneShot(&uva, &uvb, &uvc)) {
    Serial.print("UVA=");
    Serial.print(uva);
    Serial.print(" UVB=");
    Serial.print(uvb);
    Serial.print(" UVC=");
    Serial.println(uvc);
    if (uva > 10) {
      Serial.println("PASS: Got valid readings");
    } else {
      Serial.println("FAIL: Readings too low");
      all_pass = false;
    }
  } else {
    Serial.println("FAIL: oneShot() returned false");
    all_pass = false;
  }

  // Test oneShot with µW/cm²
  Serial.println("\nTesting oneShot_uWcm2():");
  float uva_uw, uvb_uw, uvc_uw;
  if (as7331.oneShot_uWcm2(&uva_uw, &uvb_uw, &uvc_uw)) {
    Serial.print("UVA=");
    Serial.print(uva_uw, 2);
    Serial.print(" UVB=");
    Serial.print(uvb_uw, 2);
    Serial.print(" UVC=");
    Serial.print(uvc_uw, 2);
    Serial.println(" µW/cm²");
    if (uva_uw > 1.0) {
      Serial.println("PASS: Got valid µW/cm² readings");
    } else {
      Serial.println("FAIL: µW/cm² readings too low");
      all_pass = false;
    }
  } else {
    Serial.println("FAIL: oneShot_uWcm2() returned false");
    all_pass = false;
  }

  Serial.println(all_pass ? "\nOverall: PASS" : "\nOverall: FAIL");
}

void loop() { delay(1000); }
