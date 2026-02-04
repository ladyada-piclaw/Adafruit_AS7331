#include <Wire.h>

#define AS7331_ADDR 0x74

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Wire.begin();
  Wire.setClock(100000); // Try slower 100kHz

  Serial.println("AS7331 I2C Diagnostic");
  Serial.println("=====================");

  // Scan for device
  Wire.beginTransmission(AS7331_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.print("Device found at 0x");
    Serial.println(AS7331_ADDR, HEX);
  } else {
    Serial.println("Device NOT found!");
    while (1)
      ;
  }

  // Try to read first 16 registers
  Serial.println("\nReading registers 0x00-0x0F:");
  for (uint8_t reg = 0; reg <= 0x0F; reg++) {
    Wire.beginTransmission(AS7331_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false); // Keep bus active

    if (err != 0) {
      Serial.print("0x");
      Serial.print(reg, HEX);
      Serial.print(": NACK (err ");
      Serial.print(err);
      Serial.println(")");
      continue;
    }

    Wire.requestFrom(AS7331_ADDR, (uint8_t)1);
    if (Wire.available()) {
      uint8_t val = Wire.read();
      Serial.print("0x");
      if (reg < 0x10)
        Serial.print("0");
      Serial.print(reg, HEX);
      Serial.print(" = 0x");
      if (val < 0x10)
        Serial.print("0");
      Serial.println(val, HEX);
    }
    delay(5);
  }

  Serial.println("\nDone.");
}

void loop() { delay(1000); }
