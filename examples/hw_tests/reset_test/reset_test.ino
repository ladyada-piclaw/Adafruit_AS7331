#include <Adafruit_AS7331.h>

Adafruit_AS7331 as7331;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("AS7331 Reset Test");

  if (!as7331.begin()) {
    Serial.println("AS7331 not found!");
    while (1) delay(10);
  }

  // Check device ID
  uint8_t id = as7331.getDeviceID();
  Serial.print("Device ID: 0x");
  Serial.println(id, HEX);
  if (id != 0x21) {
    Serial.println("FAIL: Wrong device ID");
  } else {
    Serial.println("PASS: Device ID correct");
  }

  // Set non-default values
  as7331.powerDown(true);
  as7331.setGain(AS7331_GAIN_8X);
  as7331.setIntegrationTime(AS7331_TIME_128MS);

  // Verify they took
  as7331_gain_t gain_before = as7331.getGain();
  as7331_time_t time_before = as7331.getIntegrationTime();
  Serial.print("Before reset - Gain: ");
  Serial.print(gain_before);
  Serial.print(", Time: ");
  Serial.println(time_before);

  // Reset
  Serial.println("Calling reset()...");
  as7331.reset();

  // Read back - should be defaults (GAIN=10 for 2x, TIME=6 for 64ms)
  as7331_gain_t gain_after = as7331.getGain();
  as7331_time_t time_after = as7331.getIntegrationTime();
  Serial.print("After reset - Gain: ");
  Serial.print(gain_after);
  Serial.print(", Time: ");
  Serial.println(time_after);

  bool pass = (gain_after == AS7331_GAIN_2X) &&
              (time_after == AS7331_TIME_64MS);
  Serial.println(pass ? "Overall: PASS" : "Overall: FAIL");
}

void loop() {
  delay(1000);
}
