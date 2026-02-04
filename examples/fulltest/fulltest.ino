#include <Adafruit_AS7331.h>

Adafruit_AS7331 as7331;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("AS7331 Full Test");
  Serial.println("================");

  if (!as7331.begin()) {
    Serial.println("AS7331 not found!");
    while (1) delay(10);
  }

  // Print device info
  Serial.print("Device ID: 0x");
  Serial.println(as7331.getDeviceID(), HEX);

  // Configure sensor
  as7331.powerDown(true);

  as7331.setGain(AS7331_GAIN_4X);
  as7331.setIntegrationTime(AS7331_TIME_64MS);
  as7331.setMeasurementMode(AS7331_MODE_CONT);
  as7331.setClockFrequency(AS7331_CLOCK_1024MHZ);
  as7331.setBreakTime(25); // 200µs
  as7331.setStandby(false);
  as7331.setReadyPinOpenDrain(false); // Push-pull

  // Print current configuration
  Serial.println("\nConfiguration:");
  Serial.print("  Gain: ");
  Serial.println(as7331.getGain());
  Serial.print("  Integration Time: ");
  Serial.println(as7331.getIntegrationTime());
  Serial.print("  Measurement Mode: ");
  Serial.println(as7331.getMeasurementMode());
  Serial.print("  Clock Frequency: ");
  Serial.println(as7331.getClockFrequency());
  Serial.print("  Break Time: ");
  Serial.println(as7331.getBreakTime());
  Serial.print("  Standby: ");
  Serial.println(as7331.getStandby());
  Serial.print("  READY Pin Open-Drain: ");
  Serial.println(as7331.getReadyPinOpenDrain());
  Serial.print("  Edge Count: ");
  Serial.println(as7331.getEdgeCount());

  // Start measurements
  as7331.powerDown(false);

  Serial.println("\nStarting continuous readings...\n");
}

void loop() {
  // Read raw counts
  uint16_t uva, uvb, uvc;
  if (as7331.readAllUV(&uva, &uvb, &uvc)) {
    Serial.print("Raw - UVA: ");
    Serial.print(uva);
    Serial.print("  UVB: ");
    Serial.print(uvb);
    Serial.print("  UVC: ");
    Serial.println(uvc);
  }

  // Read in µW/cm²
  float uva_uw, uvb_uw, uvc_uw;
  if (as7331.readAllUV_uWcm2(&uva_uw, &uvb_uw, &uvc_uw)) {
    Serial.print("µW/cm² - UVA: ");
    Serial.print(uva_uw, 2);
    Serial.print("  UVB: ");
    Serial.print(uvb_uw, 2);
    Serial.print("  UVC: ");
    Serial.println(uvc_uw, 2);
  }

  // Temperature
  Serial.print("Temperature: ");
  Serial.print(as7331.readTemperature(), 1);
  Serial.println(" °C");

  // Status
  uint8_t status = as7331.getStatus();
  Serial.print("Status: 0x");
  Serial.print(status, HEX);
  if (as7331.hasNewData()) Serial.print(" [NEW]");
  if (as7331.hasOverflow()) Serial.print(" [OVERFLOW]");
  if (as7331.hasLostData()) Serial.print(" [LOST]");
  Serial.println();

  Serial.println();
  delay(1000);
}
