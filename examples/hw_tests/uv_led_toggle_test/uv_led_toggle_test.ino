#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  pixels.begin();
  pixels.clear();
  pixels.show();

  if (!as7331.begin()) {
    Serial.println("AS7331 not found");
    while (1) {
      delay(10);
    }
  }

  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_2X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.powerDown(false);

  Serial.println("AS7331 UV LED toggle test starting");
}

void loop() {
  uint16_t off_uva = 0;
  uint16_t off_uvb = 0;
  uint16_t off_uvc = 0;
  uint16_t on_uva = 0;
  uint16_t on_uvb = 0;
  uint16_t on_uvc = 0;

  pixels.clear();
  pixels.show();
  delay(500);
  as7331.readAllUV(&off_uva, &off_uvb, &off_uvc);

  pixels.fill(pixels.Color(255, 255, 255));
  pixels.show();
  delay(500);
  as7331.readAllUV(&on_uva, &on_uvb, &on_uvc);

  Serial.print("OFF UVA/UVB/UVC: ");
  Serial.print(off_uva);
  Serial.print(" / ");
  Serial.print(off_uvb);
  Serial.print(" / ");
  Serial.println(off_uvc);

  Serial.print("ON  UVA/UVB/UVC: ");
  Serial.print(on_uva);
  Serial.print(" / ");
  Serial.print(on_uvb);
  Serial.print(" / ");
  Serial.println(on_uvc);

  if (on_uva > (off_uva + 50)) {
    Serial.println("PASS: UVA increased when LEDs ON");
  } else {
    Serial.println("FAIL: UVA did not increase enough");
  }

  Serial.println();
  delay(2000);
}
