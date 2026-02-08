// AS7331 READY Pin Hardware Test
// Tests READY pin behavior through measurement cycles
// Hardware: Metro Mini (AVR) with AS7331 on I2C, READY connected to D3
//
// Correct READY pin behavior (confirmed by diagnostic):
// - READY = HIGH during power-down/config state
// - READY goes LOW when entering measurement mode (powerDown(false))
// - READY stays LOW during conversion
// - READY goes HIGH when data is ready
// - READY stays HIGH until next measurement starts

#include <Adafruit_AS7331.h>

#define READY_PIN 3
#define TIMEOUT_MS 500

Adafruit_AS7331 as7331;

uint8_t testsPassed = 0;
uint8_t testsFailed = 0;

void printResult(const char *testName, bool passed) {
  Serial.print(testName);
  Serial.print(F(": "));
  if (passed) {
    Serial.println(F("PASS"));
    testsPassed++;
  } else {
    Serial.println(F("FAIL"));
    testsFailed++;
  }
}

void setup() {
  // Configure READY pin - use INPUT_PULLUP for open-drain compatibility
  pinMode(READY_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("\n========================================"));
  Serial.println(F("AS7331 READY Pin Hardware Test"));
  Serial.println(F("Settings: CMD mode, 64ms, 16x gain"));
  Serial.println(F("========================================\n"));

  // Initialize sensor
  Wire.begin();
  if (!as7331.begin()) {
    Serial.println(F("ERROR: Failed to find AS7331 sensor!"));
    while (1)
      delay(100);
  }
  Serial.println(F("AS7331 found!\n"));

  // Configure for testing: CMD mode, 64ms integration, 16x gain
  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_CMD);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setGain(AS7331_GAIN_16X);

  // ========================================
  // TEST 1: Register Configuration
  // ========================================
  Serial.println(F("--- TEST 1: Register Configuration ---"));

  // Test setReadyPinOpenDrain(true) readback
  as7331.setReadyPinOpenDrain(true);
  bool od1 = as7331.getReadyPinOpenDrain();
  printResult("setReadyPinOpenDrain(true) readback", od1 == true);

  // Test setReadyPinOpenDrain(false) readback
  as7331.setReadyPinOpenDrain(false);
  bool od2 = as7331.getReadyPinOpenDrain();
  printResult("setReadyPinOpenDrain(false) readback", od2 == false);

  Serial.println();

  // ========================================
  // TEST 2: State Transitions
  // ========================================
  Serial.println(F("--- TEST 2: State Transitions ---"));

  // Ensure we're in power-down state
  as7331.powerDown(true);
  delay(5);

  // Check READY is HIGH after power-down (config state)
  int state1 = digitalRead(READY_PIN);
  Serial.print(F("After powerDown(true): D3 = "));
  Serial.println(state1 ? "HIGH" : "LOW");
  printResult("READY HIGH in power-down state", state1 == HIGH);

  // Call powerDown(false) -> verify READY goes LOW
  as7331.powerDown(false);
  delay(1);
  int state2 = digitalRead(READY_PIN);
  Serial.print(F("After powerDown(false): D3 = "));
  Serial.println(state2 ? "HIGH" : "LOW");
  printResult("READY goes LOW when entering measurement mode", state2 == LOW);

  // Call startMeasurement() -> verify READY stays LOW
  uint32_t startTime = millis();
  as7331.startMeasurement();
  delay(1);
  int state3 = digitalRead(READY_PIN);
  Serial.print(F("After startMeasurement(): D3 = "));
  Serial.println(state3 ? "HIGH" : "LOW");
  printResult("READY stays LOW during conversion", state3 == LOW);

  // Wait for READY to go HIGH (data ready)
  bool readyWentHigh = false;
  uint32_t readyTime = 0;
  while (millis() - startTime < TIMEOUT_MS) {
    if (digitalRead(READY_PIN) == HIGH) {
      readyWentHigh = true;
      readyTime = millis() - startTime;
      break;
    }
  }

  if (readyWentHigh) {
    Serial.print(F("READY went HIGH after "));
    Serial.print(readyTime);
    Serial.println(F(" ms"));
    printResult("READY goes HIGH when data ready", true);

    // Verify READY stays HIGH until next measurement
    delay(50);
    int state4 = digitalRead(READY_PIN);
    printResult("READY stays HIGH after data ready", state4 == HIGH);
  } else {
    Serial.println(F("TIMEOUT waiting for READY to go HIGH!"));
    printResult("READY goes HIGH when data ready", false);
    printResult("READY stays HIGH after data ready", false);
  }

  // Read UV data to complete cycle
  uint16_t uva, uvb, uvc;
  as7331.readAllUV(&uva, &uvb, &uvc);
  Serial.print(F("UV data: A="));
  Serial.print(uva);
  Serial.print(F(" B="));
  Serial.print(uvb);
  Serial.print(F(" C="));
  Serial.println(uvc);

  Serial.println();

  // ========================================
  // Summary
  // ========================================
  Serial.println(F("========================================"));
  Serial.print(F("SUMMARY: "));
  Serial.print(testsPassed);
  Serial.print(F(" passed, "));
  Serial.print(testsFailed);
  Serial.println(F(" failed"));
  Serial.println(F("========================================"));

  if (testsFailed == 0) {
    Serial.println(F("\nALL TESTS PASSED!\n"));
  } else {
    Serial.println(F("\nSOME TESTS FAILED!\n"));
  }

  Serial.println(F("--- Starting scope-friendly loop (2s gaps) ---\n"));
}

void loop() {
  static uint32_t cycle = 0;
  cycle++;

  Serial.print(F("=== Cycle "));
  Serial.print(cycle);
  Serial.println(F(" ==="));

  // Ensure device is ready for measurement
  as7331.powerDown(true);
  delay(5);
  as7331.powerDown(false);
  delay(1);

  // Print state before startMeasurement
  int stateBefore = digitalRead(READY_PIN);
  Serial.print(F("Before start: D3="));
  Serial.println(stateBefore ? "HIGH" : "LOW");

  // Start measurement
  uint32_t startTime = millis();
  as7331.startMeasurement();

  // Poll for READY HIGH with timing
  bool success = false;
  uint32_t readyTime = 0;
  while (millis() - startTime < TIMEOUT_MS) {
    if (digitalRead(READY_PIN) == HIGH) {
      success = true;
      readyTime = millis() - startTime;
      break;
    }
  }

  if (success) {
    Serial.print(F("READY HIGH after "));
    Serial.print(readyTime);
    Serial.println(F(" ms"));

    // Read and print UV values
    uint16_t uva, uvb, uvc;
    as7331.readAllUV(&uva, &uvb, &uvc);
    Serial.print(F("UV: A="));
    Serial.print(uva);
    Serial.print(F(" B="));
    Serial.print(uvb);
    Serial.print(F(" C="));
    Serial.println(uvc);

    // Check behavior was correct
    bool behaviorOK = (stateBefore == LOW);
    Serial.print(F("Result: "));
    Serial.println(behaviorOK ? "PASS" : "FAIL (expected LOW before start)");
  } else {
    Serial.println(F("TIMEOUT - FAIL"));
  }

  Serial.println();
  delay(2000);
}
