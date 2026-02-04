#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

#define AS7331_ADDR 0x74

Adafruit_I2CDevice *i2c_dev;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("BusIO Test for AS7331");
  Serial.println("=====================");

  i2c_dev = new Adafruit_I2CDevice(AS7331_ADDR);
  if (!i2c_dev->begin()) {
    Serial.println("I2C device not found!");
    while (1)
      ;
  }
  Serial.println("I2C device found.");

  // Method 1: Direct read with buffer
  Serial.println("\nMethod 1: Direct i2c_dev->write_then_read()");
  uint8_t reg = 0x02;
  uint8_t val = 0;
  if (i2c_dev->write_then_read(&reg, 1, &val, 1)) {
    Serial.print("Reg 0x02 = 0x");
    Serial.println(val, HEX);
  } else {
    Serial.println("write_then_read FAILED");
  }

  // Method 2: BusIO_Register with explicit 1-byte width
  Serial.println("\nMethod 2: BusIO_Register (8-bit, width=1)");
  Adafruit_BusIO_Register agen8(i2c_dev, 0x02, 1);
  uint8_t val8 = 0;
  if (agen8.read(&val8)) {
    Serial.print("Reg 0x02 = 0x");
    Serial.println(val8, HEX);
  } else {
    Serial.println("BusIO_Register read FAILED");
  }

  // Method 3: BusIO_Register default (might be 2 bytes?)
  Serial.println("\nMethod 3: BusIO_Register (default constructor)");
  Adafruit_BusIO_Register agen_def(i2c_dev, 0x02);
  uint8_t val_def = 0;
  if (agen_def.read(&val_def)) {
    Serial.print("Reg 0x02 = 0x");
    Serial.println(val_def, HEX);
  } else {
    Serial.println("BusIO_Register default read FAILED");
  }

  // Method 4: Read as 16-bit
  Serial.println("\nMethod 4: BusIO_Register (16-bit)");
  Adafruit_BusIO_Register agen16(i2c_dev, 0x02, 2, LSBFIRST);
  uint16_t val16 = 0;
  if (agen16.read(&val16)) {
    Serial.print("Reg 0x02 (16-bit) = 0x");
    Serial.println(val16, HEX);
  } else {
    Serial.println("16-bit read FAILED");
  }

  Serial.println("\nDone.");
}

void loop() { delay(1000); }
