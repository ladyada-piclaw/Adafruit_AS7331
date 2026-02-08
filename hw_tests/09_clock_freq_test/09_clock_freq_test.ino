/**
 * AS7331 Clock Frequency Functional Test
 *
 * Tests that clock frequency actually affects measurement timing,
 * not just register readback.
 *
 * Approach:
 * - Use CMD mode with fixed integration time setting (64ms nominal)
 * - Measure actual time from startMeasurement() to READY HIGH
 * - Test with 1.024MHz vs 8.192MHz clock
 * - Higher clock = faster measurement (8x clock should be noticeably faster)
 *
 * The integration time setting specifies ADC cycles, so actual wall-clock
 * time depends on the clock frequency.
 *
 * Hardware: Metro Mini, AS7331 on I2C, READY→D3
 */

#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define READY_PIN 3
#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60
#define TIMEOUT_MS 2000
#define NUM_SAMPLES 3

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

// Measure time for a single CMD mode measurement
// Returns time in microseconds, or 0 on timeout
uint32_t measureOneShotTime() {
  // Ensure we're ready
  as7331.powerDown(false);
  delay(5);

  // Start measurement and time it
  uint32_t start = micros();
  as7331.startMeasurement();

  // Wait for READY HIGH
  uint32_t deadline = millis() + TIMEOUT_MS;
  while (digitalRead(READY_PIN) == LOW) {
    if (millis() > deadline) {
      Serial.println(F("  Timeout waiting for READY"));
      return 0;
    }
  }

  uint32_t elapsed = micros() - start;

  // Read data to clear state
  uint16_t uva, uvb, uvc;
  as7331.readAllUV(&uva, &uvb, &uvc);

  return elapsed;
}

// Measure average time over multiple samples
uint32_t measureAverageTime(uint8_t numSamples) {
  uint32_t total = 0;
  for (uint8_t i = 0; i < numSamples; i++) {
    uint32_t t = measureOneShotTime();
    if (t == 0)
      return 0;
    total += t;
    delay(10);
  }
  return total / numSamples;
}

void setup() {
  pinMode(READY_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("\n========================================"));
  Serial.println(F("AS7331 Clock Frequency Functional Test"));
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
  as7331.setClockFrequency(AS7331_CLOCK_1024MHZ);
  as7331_clock_t read1 = as7331.getClockFrequency();
  printResult("1.024MHz readback", read1 == AS7331_CLOCK_1024MHZ);

  as7331.setClockFrequency(AS7331_CLOCK_8192MHZ);
  as7331_clock_t read8 = as7331.getClockFrequency();
  printResult("8.192MHz readback", read8 == AS7331_CLOCK_8192MHZ);

  Serial.println();

  // ========================================
  // TEST 2: Timing with 1.024MHz clock (slowest)
  // ========================================
  Serial.println(F("--- TEST 2: Timing with 1.024MHz clock ---"));

  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_CMD);
  as7331.setIntegrationTime(AS7331_TIME_64MS); // Nominal 64ms at 1.024MHz
  as7331.setGain(AS7331_GAIN_16X);
  as7331.setClockFrequency(AS7331_CLOCK_1024MHZ);
  as7331.powerDown(false);
  delay(10);

  uint32_t time1024 = measureAverageTime(NUM_SAMPLES);
  Serial.print(F("  Average measurement time: "));
  Serial.print(time1024);
  Serial.print(F(" us ("));
  Serial.print(time1024 / 1000);
  Serial.println(F(" ms)"));

  // ========================================
  // TEST 3: Timing with 8.192MHz clock (fastest)
  // ========================================
  Serial.println(F("\n--- TEST 3: Timing with 8.192MHz clock ---"));

  as7331.powerDown(true);
  as7331.setClockFrequency(AS7331_CLOCK_8192MHZ);
  as7331.powerDown(false);
  delay(10);

  uint32_t time8192 = measureAverageTime(NUM_SAMPLES);
  Serial.print(F("  Average measurement time: "));
  Serial.print(time8192);
  Serial.print(F(" us ("));
  Serial.print(time8192 / 1000);
  Serial.println(F(" ms)"));

  // ========================================
  // TEST 4: Verify timing difference
  // ========================================
  Serial.println(F("\n--- TEST 4: Timing Comparison ---"));

  if (time1024 > 0 && time8192 > 0) {
    // Calculate ratio
    float ratio = (float)time1024 / (float)time8192;
    Serial.print(F("Time ratio (1.024MHz / 8.192MHz): "));
    Serial.println(ratio, 2);

    // With 8x clock, we expect ~8x faster measurement
    // Accept any significant speedup (>2x) as proof clock affects timing
    // Note: actual ratio may vary due to measurement overhead, dividers, etc.
    bool ratioReasonable = (ratio > 2.0);
    printResult("Higher clock is significantly faster (>2x)", ratioReasonable);

    // Verify the faster clock is actually faster
    bool fasterIsFaster = time8192 < time1024;
    printResult("8.192MHz is faster than 1.024MHz", fasterIsFaster);

    // Sanity: both times should be reasonable (>1ms, <500ms)
    bool times_sane = (time1024 > 1000) && (time1024 < 500000) &&
                      (time8192 > 1000) && (time8192 < 500000);
    printResult("Timing values are sane", times_sane);
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
