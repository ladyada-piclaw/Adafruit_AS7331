/**
 * AS7331 Measurement Mode Functional Test
 *
 * Tests that each measurement mode behaves correctly,
 * not just register readback.
 *
 * Approach:
 * - CONT mode: Start measurement, verify multiple readings come automatically
 * - CMD mode: startMeasurement() produces exactly ONE reading, then stops
 * - SYNS mode: Measurement waits for SYNC rising edge to start
 * - SYND mode: Just verify it enters the mode (detailed in edge_count test)
 *
 * Hardware: Metro Mini, AS7331 on I2C, SYNC→D2, READY→D3
 */

#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define SYNC_PIN 2
#define READY_PIN 3
#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60
#define TIMEOUT_MS 500

Adafruit_AS7331 as7331;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

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

bool isReadyHigh() { return digitalRead(READY_PIN) == HIGH; }

bool waitForReadyHigh(uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (isReadyHigh())
      return true;
    delay(1);
  }
  return false;
}

bool waitForReadyLow(uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (!isReadyHigh())
      return true;
    delay(1);
  }
  return false;
}

// Generate a single rising edge pulse on SYNC pin
void pulseSync() {
  digitalWrite(SYNC_PIN, HIGH);
  delay(50);
  digitalWrite(SYNC_PIN, LOW);
  delay(50);
}

void setup() {
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  pinMode(READY_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("\n========================================"));
  Serial.println(F("AS7331 Measurement Mode Functional Test"));
  Serial.println(F("Tests actual mode behavior"));
  Serial.println(F("========================================\n"));

  // NeoPixels for consistent light source
  pixels.begin();
  pixels.setBrightness(255);
  pixels.fill(pixels.Color(255, 255, 255));
  pixels.show();

  if (!as7331.begin()) {
    Serial.println(F("ERROR: AS7331 not found!"));
    while (1)
      delay(100);
  }
  Serial.println(F("AS7331 found!\n"));

  // ========================================
  // TEST 1: Register readback verification
  // ========================================
  Serial.println(F("--- TEST 1: Register Readback ---"));

  as7331.powerDown(true);

  as7331.setMeasurementMode(AS7331_MODE_CONT);
  printResult("CONT mode readback",
              as7331.getMeasurementMode() == AS7331_MODE_CONT);

  as7331.setMeasurementMode(AS7331_MODE_CMD);
  printResult("CMD mode readback",
              as7331.getMeasurementMode() == AS7331_MODE_CMD);

  as7331.setMeasurementMode(AS7331_MODE_SYNS);
  printResult("SYNS mode readback",
              as7331.getMeasurementMode() == AS7331_MODE_SYNS);

  as7331.setMeasurementMode(AS7331_MODE_SYND);
  printResult("SYND mode readback",
              as7331.getMeasurementMode() == AS7331_MODE_SYND);

  Serial.println();

  // ========================================
  // TEST 2: CONT mode - multiple automatic readings
  // ========================================
  Serial.println(F("--- TEST 2: CONT Mode Behavior ---"));
  Serial.println(F("Expect: Multiple READY pulses automatically"));

  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.setIntegrationTime(AS7331_TIME_16MS);
  as7331.setGain(AS7331_GAIN_16X);
  as7331.powerDown(false);
  as7331.startMeasurement();

  // Count READY HIGH transitions over ~500ms
  uint8_t readyCount = 0;
  uint32_t startTime = millis();
  bool lastState = isReadyHigh();

  while (millis() - startTime < 500) {
    bool currentState = isReadyHigh();
    // Count LOW->HIGH transitions (new data ready)
    if (!lastState && currentState) {
      readyCount++;
      // Read data to acknowledge
      uint16_t uva, uvb, uvc;
      as7331.readAllUV(&uva, &uvb, &uvc);
    }
    lastState = currentState;
    delay(1);
  }

  as7331.stopMeasurement();

  Serial.print(F("  Detected "));
  Serial.print(readyCount);
  Serial.println(F(" READY pulses in 500ms"));

  // With 16ms integration, expect ~20-30 readings in 500ms
  // Accept >=3 as proof of continuous operation
  bool contWorks = readyCount >= 3;
  printResult("CONT mode produces multiple automatic readings", contWorks);

  Serial.println();

  // ========================================
  // TEST 3: CMD mode - single reading only
  // ========================================
  Serial.println(F("--- TEST 3: CMD Mode Behavior ---"));
  Serial.println(F("Expect: Exactly ONE reading per startMeasurement()"));

  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_CMD);
  as7331.setIntegrationTime(AS7331_TIME_16MS);
  as7331.powerDown(false);

  // Start one measurement
  as7331.startMeasurement();

  // Wait for first READY HIGH
  bool firstReady = waitForReadyHigh(TIMEOUT_MS);
  Serial.print(F("  First READY: "));
  Serial.println(firstReady ? "detected" : "timeout");

  if (firstReady) {
    // Read data
    uint16_t uva, uvb, uvc;
    as7331.readAllUV(&uva, &uvb, &uvc);
    Serial.print(F("  First UV: A="));
    Serial.print(uva);
    Serial.print(F(" B="));
    Serial.print(uvb);
    Serial.print(F(" C="));
    Serial.println(uvc);

    // Wait for READY to go LOW, then check if it comes back HIGH
    // (it shouldn't in CMD mode without another startMeasurement)
    waitForReadyLow(100);

    // Wait and see if another measurement starts automatically
    Serial.println(F("  Waiting 200ms for spontaneous second measurement..."));
    bool secondReady = waitForReadyHigh(200);

    if (!secondReady) {
      Serial.println(F("  No second measurement (correct!)"));
      printResult("CMD mode stops after one reading", true);
    } else {
      Serial.println(F("  Unexpected second measurement!"));
      printResult("CMD mode stops after one reading", false);
    }
  } else {
    printResult("CMD mode first reading", false);
  }

  Serial.println();

  // ========================================
  // TEST 4: SYNS mode - waits for SYNC edge
  // ========================================
  Serial.println(F("--- TEST 4: SYNS Mode Behavior ---"));
  Serial.println(F("Expect: Measurement waits for SYNC rising edge"));

  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_SYNS);
  as7331.setIntegrationTime(AS7331_TIME_16MS);
  as7331.powerDown(false);

  // Ensure SYNC is LOW
  digitalWrite(SYNC_PIN, LOW);
  delay(10);

  // Start measurement
  as7331.startMeasurement();

  // Check READY state - should stay LOW (waiting for sync)
  delay(50);
  bool readyBeforeSync = isReadyHigh();
  Serial.print(F("  READY before SYNC pulse: "));
  Serial.println(readyBeforeSync ? "HIGH (unexpected)" : "LOW (expected)");

  // Wait 100ms - should still be waiting
  delay(100);
  bool stillWaiting = !isReadyHigh();
  Serial.print(F("  Still waiting after 100ms: "));
  Serial.println(stillWaiting ? "yes" : "no");
  printResult("SYNS waits for SYNC edge", stillWaiting && !readyBeforeSync);

  // Now send SYNC pulse
  Serial.println(F("  Sending SYNC pulse..."));
  pulseSync();

  // Wait for measurement to complete
  bool synsCompleted = waitForReadyHigh(TIMEOUT_MS);
  Serial.print(F("  READY after SYNC pulse: "));
  Serial.println(synsCompleted ? "HIGH" : "timeout");
  printResult("SYNS starts after SYNC edge", synsCompleted);

  if (synsCompleted) {
    uint16_t uva, uvb, uvc;
    as7331.readAllUV(&uva, &uvb, &uvc);
    Serial.print(F("  UV data: A="));
    Serial.print(uva);
    Serial.print(F(" B="));
    Serial.print(uvb);
    Serial.print(F(" C="));
    Serial.println(uvc);
  }

  Serial.println();

  // ========================================
  // TEST 5: SYND mode - basic verification
  // ========================================
  Serial.println(F("--- TEST 5: SYND Mode Basic Check ---"));
  Serial.println(F("(Detailed edge counting tested in 11_edge_count_test)"));

  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_SYND);
  as7331.setEdgeCount(1);
  as7331.setIntegrationTime(AS7331_TIME_16MS);
  as7331.powerDown(false);

  digitalWrite(SYNC_PIN, LOW);
  delay(10);

  as7331.startMeasurement();

  // Send one pulse (edge_count=1)
  pulseSync();

  bool syndWorks = waitForReadyHigh(TIMEOUT_MS);
  Serial.print(F("  SYND with edge_count=1: "));
  Serial.println(syndWorks ? "works" : "timeout");
  printResult("SYND mode responds to SYNC edges", syndWorks);

  if (syndWorks) {
    uint16_t uva, uvb, uvc;
    as7331.readAllUV(&uva, &uvb, &uvc);
    Serial.print(F("  UV data: A="));
    Serial.print(uva);
    Serial.print(F(" B="));
    Serial.print(uvb);
    Serial.print(F(" C="));
    Serial.println(uvc);
  }

  // ========================================
  // Summary
  // ========================================
  Serial.println(F("\n========================================"));
  Serial.print(F("SUMMARY: "));
  Serial.print(testsPassed);
  Serial.print(F(" passed, "));
  Serial.print(testsFailed);
  Serial.println(F(" failed"));
  Serial.println(F("========================================"));

  if (testsFailed == 0) {
    Serial.println(F("Overall: PASS"));
  } else {
    Serial.println(F("Overall: FAIL"));
  }
}

void loop() {}
