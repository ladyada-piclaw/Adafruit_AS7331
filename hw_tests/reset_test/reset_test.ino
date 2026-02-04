#include <Adafruit_AS7331.h>
#include <Wire.h>

Adafruit_AS7331 as7331;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("AS7331 reset test");
  if (!as7331.begin()) {
    Serial.println("FAIL: begin()");
    return;
  }

  bool ok = true;

  if (!as7331.setGain(AS7331_GAIN_8X)) {
    Serial.println("FAIL: setGain");
    ok = false;
  }
  if (!as7331.setIntegrationTime(AS7331_TIME_128MS)) {
    Serial.println("FAIL: setIntegrationTime");
    ok = false;
  }

  as7331_gain_t gain = as7331.getGain();
  as7331_time_t time = as7331.getIntegrationTime();
  if (gain != AS7331_GAIN_8X || time != AS7331_TIME_128MS) {
    Serial.println("FAIL: readback before reset");
    ok = false;
  }

  if (!as7331.reset()) {
    Serial.println("FAIL: reset");
    ok = false;
  }

  gain = as7331.getGain();
  time = as7331.getIntegrationTime();

  if (gain != AS7331_GAIN_2X || time != AS7331_TIME_64MS) {
    Serial.print("FAIL: defaults after reset, gain=");
    Serial.print(gain);
    Serial.print(" time=");
    Serial.println(time);
    ok = false;
  }

  if (ok) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }
}

void loop() { delay(1000); }
