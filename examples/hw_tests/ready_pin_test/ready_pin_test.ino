#include <Adafruit_AS7331.h>
#include <Wire.h>

Adafruit_AS7331 as7331;

static const uint8_t READY_PIN = 3;

static bool waitForPinState(uint8_t pin, uint8_t state, uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (digitalRead(pin) == state) {
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("AS7331 READY pin test");

  if (!as7331.begin()) {
    Serial.println("FAIL: AS7331 not found");
    while (1) {
      delay(10);
    }
  }

  bool overall_pass = true;

  bool pass1 = as7331.setReadyPinOpenDrain(false) &&
               (as7331.getReadyPinOpenDrain() == false);
  Serial.print("Test 1 (push-pull mode): ");
  Serial.println(pass1 ? "PASS" : "FAIL");
  overall_pass &= pass1;

  bool pass2 = as7331.setReadyPinOpenDrain(true) &&
               (as7331.getReadyPinOpenDrain() == true);
  Serial.print("Test 2 (open-drain mode): ");
  Serial.println(pass2 ? "PASS" : "FAIL");
  overall_pass &= pass2;

  as7331.setGain(AS7331_GAIN_2X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);

  pinMode(READY_PIN, INPUT_PULLUP);

  bool saw_low = waitForPinState(READY_PIN, LOW, 2000);
  bool saw_high = waitForPinState(READY_PIN, HIGH, 2000);

  bool pass3 = saw_low && saw_high;
  Serial.print("Test 3 (READY toggles): ");
  Serial.println(pass3 ? "PASS" : "FAIL");
  overall_pass &= pass3;

  Serial.print("Overall: ");
  Serial.println(overall_pass ? "PASS" : "FAIL");
}

void loop() {
  delay(1000);
}
