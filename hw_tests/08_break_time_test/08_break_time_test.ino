/**
 * AS7331 Break Time Functional Test
 *
 * Tests that break_time setting actually affects the inter-measurement gap
 * in CONT mode, not just register readback.
 *
 * Approach:
 * - Use CONT mode with short integration time (16ms)
 * - Measure total time for N complete measurement cycles
 * - Compare break_time=0 vs break_time=255
 * - With 10 cycles, 255*8us*10 = ~20ms expected difference
 *
 * Hardware: Metro Mini, AS7331 on I2C, READY→D3
 */

#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define READY_PIN 3
#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60
#define TIMEOUT_MS 500
#define NUM_CYCLES 10

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

// Wait for READY to transition to target state
bool waitForReady(bool targetHigh, uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (digitalRead(READY_PIN) == (targetHigh ? HIGH : LOW))
      return true;
  }
  return false;
}

// Measure total time for N complete measurement cycles in CONT mode
// Returns time in microseconds, or 0 on error
uint32_t measureNCycles(uint8_t numCycles) {
  // Wait for first READY HIGH to synchronize
  if (!waitForReady(true, TIMEOUT_MS)) {
    Serial.println(F("  Timeout waiting for first READY HIGH"));
    return 0;
  }

  // Read data to acknowledge
  uint16_t uva, uvb, uvc;
  as7331.readAllUV(&uva, &uvb, &uvc);

  // Start timing
  uint32_t startTime = micros();
  uint8_t cycles = 0;

  while (cycles < numCycles) {
    // Wait for READY LOW (next measurement starting)
    if (!waitForReady(false, TIMEOUT_MS)) {
      Serial.println(F("  Timeout waiting for READY LOW"));
      return 0;
    }

    // Wait for READY HIGH (measurement complete)
    if (!waitForReady(true, TIMEOUT_MS)) {
      Serial.println(F("  Timeout waiting for READY HIGH"));
      return 0;
    }

    // Read data
    as7331.readAllUV(&uva, &uvb, &uvc);
    cycles++;
  }

  return micros() - startTime;
}

void setup() {
  pinMode(READY_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("\n========================================"));
  Serial.println(F("AS7331 Break Time Functional Test"));
  Serial.println(F("Tests actual timing, not just register readback"));
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
  as7331.setBreakTime(0);
  uint8_t read0 = as7331.getBreakTime();
  printResult("break_time=0 readback", read0 == 0);

  as7331.setBreakTime(255);
  uint8_t read255 = as7331.getBreakTime();
  printResult("break_time=255 readback", read255 == 255);

  Serial.println();

  // ========================================
  // TEST 2: Timing with break_time=0
  // ========================================
  Serial.println(F("--- TEST 2: Timing with break_time=0 ---"));
  Serial.print(F("Measuring "));
  Serial.print(NUM_CYCLES);
  Serial.println(F(" cycles..."));

  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.setIntegrationTime(AS7331_TIME_16MS);
  as7331.setGain(AS7331_GAIN_16X);
  as7331.setClockFrequency(AS7331_CLOCK_1024MHZ);
  as7331.setBreakTime(0);
  as7331.powerDown(false);
  as7331.startMeasurement();

  delay(100); // Let CONT mode settle

  uint32_t totalTime0 = measureNCycles(NUM_CYCLES);
  Serial.print(F("  Total time for "));
  Serial.print(NUM_CYCLES);
  Serial.print(F(" cycles: "));
  Serial.print(totalTime0 / 1000);
  Serial.println(F(" ms"));

  as7331.stopMeasurement();
  as7331.powerDown(true);
  delay(50);

  // ========================================
  // TEST 3: Timing with break_time=255
  // ========================================
  Serial.println(F("\n--- TEST 3: Timing with break_time=255 ---"));
  Serial.print(F("Measuring "));
  Serial.print(NUM_CYCLES);
  Serial.println(F(" cycles..."));

  as7331.setBreakTime(255);
  as7331.powerDown(false);
  as7331.startMeasurement();

  delay(100); // Let CONT mode settle

  uint32_t totalTime255 = measureNCycles(NUM_CYCLES);
  Serial.print(F("  Total time for "));
  Serial.print(NUM_CYCLES);
  Serial.print(F(" cycles: "));
  Serial.print(totalTime255 / 1000);
  Serial.println(F(" ms"));

  as7331.stopMeasurement();

  // ========================================
  // TEST 4: Verify timing difference
  // ========================================
  Serial.println(F("\n--- TEST 4: Timing Comparison ---"));

  if (totalTime0 > 0 && totalTime255 > 0) {
    int32_t diff = (int32_t)totalTime255 - (int32_t)totalTime0;
    Serial.print(F("Time difference: "));
    Serial.print(diff / 1000);
    Serial.println(F(" ms"));

    // Expected: ~20ms (255 * 8us * 10 cycles = 20.4ms)
    // Accept anything >5ms as meaningful (allows for noise)
    bool timingDifferent = diff > 5000;

    Serial.print(F("Expected ~20ms difference, got "));
    Serial.print(diff / 1000);
    Serial.println(F(" ms"));

    printResult("break_time=255 takes measurably longer", timingDifferent);

    // Sanity: difference should be reasonable (<100ms)
    bool reasonableDiff = diff < 100000;
    printResult("Timing difference is reasonable", reasonableDiff);

    // Per-cycle timing
    Serial.print(F("Per-cycle avg with break=0: "));
    Serial.print(totalTime0 / NUM_CYCLES);
    Serial.println(F(" us"));
    Serial.print(F("Per-cycle avg with break=255: "));
    Serial.print(totalTime255 / NUM_CYCLES);
    Serial.println(F(" us"));
  } else {
    Serial.println(F("ERROR: Could not measure timing"));
    printResult("Timing measurement", false);
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
