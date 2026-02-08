/**
 * AS7331 Edge Count Functional Test
 *
 * Tests that edge_count setting actually controls SYND mode behavior,
 * not just register readback.
 *
 * Approach:
 * - Set SYND mode with edge_count=2
 * - Pulse SYNC pin twice, verify measurement completes
 * - Set edge_count=4
 * - Pulse SYNC pin twice, verify measurement does NOT complete yet
 * - Pulse 2 more times, verify measurement completes
 *
 * Hardware: Metro Mini, AS7331 on I2C, SYNC→D2, READY→D3
 */

#include <Adafruit_AS7331.h>
#include <Adafruit_NeoPixel.h>

#define SYNC_PIN 2
#define READY_PIN 3
#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 60
#define TIMEOUT_MS 2000

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

// Generate a single rising edge pulse on SYNC pin
// Using longer delays like the working sync_pulse_test
void pulseSync() {
  digitalWrite(SYNC_PIN, HIGH);
  delay(100);
  digitalWrite(SYNC_PIN, LOW);
  delay(100);
}

// Check if READY is HIGH (measurement complete)
bool isReadyHigh() { return digitalRead(READY_PIN) == HIGH; }

// Wait for READY HIGH with timeout, returns true if it went high
bool waitForReadyHigh(uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (isReadyHigh())
      return true;
    delay(1);
  }
  return false;
}

// Configure SYND mode with given edge count
void configureSYND(uint8_t edgeCount) {
  as7331.powerDown(true);
  as7331.setMeasurementMode(AS7331_MODE_SYND);
  as7331.setEdgeCount(edgeCount);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setGain(AS7331_GAIN_16X);
  as7331.powerDown(false);

  // Ensure SYNC starts LOW
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
  Serial.println(F("AS7331 Edge Count Functional Test"));
  Serial.println(F("Tests actual SYND mode edge counting"));
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
  as7331.setEdgeCount(2);
  uint8_t read2 = as7331.getEdgeCount();
  printResult("edge_count=2 readback", read2 == 2);

  as7331.setEdgeCount(4);
  uint8_t read4 = as7331.getEdgeCount();
  printResult("edge_count=4 readback", read4 == 4);

  Serial.println();

  // ========================================
  // TEST 2: edge_count=2 completes after 2 pulses
  // ========================================
  Serial.println(F("--- TEST 2: edge_count=2, send 2 pulses ---"));

  configureSYND(2);
  as7331.startMeasurement();
  delay(50);

  Serial.println(F("  Starting measurement..."));
  Serial.print(F("  READY initial: "));
  Serial.println(isReadyHigh() ? "HIGH" : "LOW");

  Serial.println(F("  Sending pulse 1..."));
  pulseSync();
  Serial.print(F("  READY: "));
  Serial.println(isReadyHigh() ? "HIGH" : "LOW");

  Serial.println(F("  Sending pulse 2..."));
  pulseSync();

  // Wait for measurement to complete
  delay(200);
  bool completedWith2 = isReadyHigh();
  Serial.print(F("  READY after wait: "));
  Serial.println(completedWith2 ? "HIGH (complete)" : "LOW");
  printResult("Measurement completes after 2 pulses", completedWith2);

  // Read data
  uint16_t uva, uvb, uvc;
  as7331.readAllUV(&uva, &uvb, &uvc);
  Serial.print(F("  UV data: A="));
  Serial.print(uva);
  Serial.print(F(" B="));
  Serial.print(uvb);
  Serial.print(F(" C="));
  Serial.println(uvc);

  Serial.println();

  // ========================================
  // TEST 3: edge_count=4, verify 2 pulses NOT enough
  // ========================================
  Serial.println(F("--- TEST 3: edge_count=4, send only 2 pulses ---"));

  configureSYND(4);
  as7331.startMeasurement();
  delay(50);

  Serial.println(F("  Starting measurement..."));

  Serial.println(F("  Sending pulse 1..."));
  pulseSync();

  Serial.println(F("  Sending pulse 2..."));
  pulseSync();

  // Quick check - should NOT be complete yet
  delay(200);
  bool notCompleteYet = !isReadyHigh();
  Serial.print(F("  READY after 2/4 pulses: "));
  Serial.println(isReadyHigh() ? "HIGH (unexpected!)" : "LOW (expected)");
  printResult("Measurement NOT complete after only 2/4 pulses", notCompleteYet);

  Serial.println();

  // ========================================
  // TEST 4: edge_count=4, complete with 2 more pulses
  // ========================================
  Serial.println(F("--- TEST 4: Continue with 2 more pulses ---"));

  Serial.println(F("  Sending pulse 3..."));
  pulseSync();
  Serial.print(F("  READY: "));
  Serial.println(isReadyHigh() ? "HIGH" : "LOW");

  Serial.println(F("  Sending pulse 4..."));
  pulseSync();

  delay(200);
  bool completedWith4 = isReadyHigh();
  Serial.print(F("  READY after 4 pulses: "));
  Serial.println(completedWith4 ? "HIGH (complete)" : "LOW");
  printResult("Measurement completes after 4 total pulses", completedWith4);

  // Read data
  as7331.readAllUV(&uva, &uvb, &uvc);
  Serial.print(F("  UV data: A="));
  Serial.print(uva);
  Serial.print(F(" B="));
  Serial.print(uvb);
  Serial.print(F(" C="));
  Serial.println(uvc);

  Serial.println();

  // ========================================
  // TEST 5: Functional verification via data
  // ========================================
  Serial.println(F("--- TEST 5: Data validity check ---"));

  // Even if READY timing is tricky, verify we get data after correct pulses
  configureSYND(2);
  as7331.startMeasurement();
  delay(50);

  pulseSync();
  pulseSync();
  delay(500); // Long wait

  as7331.readAllUV(&uva, &uvb, &uvc);
  Serial.print(F("  2 pulses, edge_count=2: UV A="));
  Serial.print(uva);
  Serial.print(F(" B="));
  Serial.print(uvb);
  Serial.print(F(" C="));
  Serial.println(uvc);

  bool dataValid2 = (uva > 0 || uvb > 0 || uvc > 0);
  printResult("Got valid data with matching pulse count", dataValid2);

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
