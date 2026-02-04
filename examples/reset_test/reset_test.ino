#include <Wire.h>

#define AS7331_ADDR 0x74
#define REG_OSR 0x00
#define REG_AGEN 0x02

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Wire.begin();
  
  // Software reset: write 0x0A to OSR (SW_RES=1, DOS=010)
  Serial.println("Sending software reset...");
  Wire.beginTransmission(AS7331_ADDR);
  Wire.write(REG_OSR);
  Wire.write(0x0A);  // SW_RES=1, DOS=010
  Wire.endTransmission();
  
  delay(100);  // Wait for reset
  
  // Read AGEN
  Serial.println("Reading AGEN register...");
  Wire.beginTransmission(AS7331_ADDR);
  Wire.write(REG_AGEN);
  Wire.endTransmission();
  
  Wire.requestFrom(AS7331_ADDR, 1);
  if (Wire.available()) {
    uint8_t agen = Wire.read();
    Serial.print("AGEN = 0x");
    Serial.println(agen, HEX);
    if (agen == 0x21) {
      Serial.println("SUCCESS: AS7331 found!");
    } else {
      Serial.print("FAIL: Expected 0x21, got 0x");
      Serial.println(agen, HEX);
    }
  } else {
    Serial.println("FAIL: No response from sensor");
  }
}

void loop() {
  delay(1000);
}
