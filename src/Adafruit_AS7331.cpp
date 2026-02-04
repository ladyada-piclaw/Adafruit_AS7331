#include "Adafruit_AS7331.h"

Adafruit_AS7331::Adafruit_AS7331() {}

bool Adafruit_AS7331::begin(TwoWire *wire, uint8_t addr) {
  if (_i2c_dev) {
    delete _i2c_dev;
    _i2c_dev = nullptr;
  }

  _i2c_dev = new Adafruit_I2CDevice(addr, wire);
  if (!_i2c_dev->begin()) {
    return false;
  }

  // Software reset to ensure we're in Configuration State.
  // The sensor retains state across Arduino resets, so we must
  // reset it to read AGEN (which shares address with MRES1).
  // Write OSR = 0x0A (SW_RES=1, DOS=010 for config state)
  Adafruit_BusIO_Register osr(_i2c_dev, AS7331_REG_OSR);
  if (!osr.write(0x0A)) {
    return false;
  }
  delay(10); // Wait for reset to complete

  // Verify we're in config state by reading AGEN (0x02)
  Adafruit_BusIO_Register agen(_i2c_dev, AS7331_REG_AGEN);
  uint8_t part_id = 0;
  if (!agen.read(&part_id)) {
    return false;
  }

  return (part_id == AS7331_PART_ID);
}

bool Adafruit_AS7331::powerDown(bool pd) {
  Adafruit_BusIO_Register osr(_i2c_dev, AS7331_REG_OSR);
  Adafruit_BusIO_RegisterBits ss_bit(&osr, 1, 7);
  Adafruit_BusIO_RegisterBits pd_bit(&osr, 1, 6);
  Adafruit_BusIO_RegisterBits dos_bits(&osr, 3, 0);

  if (!pd_bit.write(pd)) {
    return false;
  }

  if (pd) {
    if (!ss_bit.write(false)) {
      return false;
    }
    if (!dos_bits.write(0x02)) {
      return false;
    }
  } else {
    if (!dos_bits.write(0x03)) {
      return false;
    }
    if (!ss_bit.write(true)) {
      return false;
    }
    delay(2);
  }

  return true;
}

bool Adafruit_AS7331::setMeasurementMode(as7331_mode_t mode) {
  Adafruit_BusIO_Register creg3(_i2c_dev, AS7331_REG_CREG3);
  Adafruit_BusIO_RegisterBits mode_bits(&creg3, 2, 6);
  return mode_bits.write(mode);
}

bool Adafruit_AS7331::setGain(as7331_gain_t gain) {
  Adafruit_BusIO_Register creg1(_i2c_dev, AS7331_REG_CREG1);
  Adafruit_BusIO_RegisterBits gain_bits(&creg1, 4, 4);
  return gain_bits.write(gain);
}

as7331_gain_t Adafruit_AS7331::getGain(void) {
  Adafruit_BusIO_Register creg1(_i2c_dev, AS7331_REG_CREG1);
  Adafruit_BusIO_RegisterBits gain_bits(&creg1, 4, 4);
  return (as7331_gain_t)gain_bits.read();
}

bool Adafruit_AS7331::setIntegrationTime(as7331_time_t time) {
  Adafruit_BusIO_Register creg1(_i2c_dev, AS7331_REG_CREG1);
  Adafruit_BusIO_RegisterBits time_bits(&creg1, 4, 0);
  return time_bits.write(time);
}

as7331_time_t Adafruit_AS7331::getIntegrationTime(void) {
  Adafruit_BusIO_Register creg1(_i2c_dev, AS7331_REG_CREG1);
  Adafruit_BusIO_RegisterBits time_bits(&creg1, 4, 0);
  return (as7331_time_t)time_bits.read();
}

uint16_t Adafruit_AS7331::readUVA(void) {
  uint16_t value = 0;
  readRegister(AS7331_REG_MRES1, &value);
  return value;
}

uint16_t Adafruit_AS7331::readUVB(void) {
  uint16_t value = 0;
  readRegister(AS7331_REG_MRES2, &value);
  return value;
}

uint16_t Adafruit_AS7331::readUVC(void) {
  uint16_t value = 0;
  readRegister(AS7331_REG_MRES3, &value);
  return value;
}

bool Adafruit_AS7331::readAllUV(uint16_t *uva, uint16_t *uvb, uint16_t *uvc) {
  uint8_t buffer[6] = {0};
  if (!readRegisters(AS7331_REG_MRES1, buffer, sizeof(buffer))) {
    return false;
  }

  if (uva) {
    *uva = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
  }
  if (uvb) {
    *uvb = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
  }
  if (uvc) {
    *uvc = (uint16_t)buffer[4] | ((uint16_t)buffer[5] << 8);
  }

  return true;
}

float Adafruit_AS7331::readTemperature(void) {
  uint16_t raw = 0;
  readRegister(AS7331_REG_TEMP, &raw);
  raw &= 0x0FFF;
  return (raw * 0.05f) - 66.9f;
}

bool Adafruit_AS7331::isDataReady(void) {
  uint16_t status = 0;
  if (!readRegister(AS7331_REG_OSR, &status)) {
    return false;
  }

  bool not_ready = (status >> 10) & 0x1;
  return !not_ready;
}

bool Adafruit_AS7331::readRegister(uint8_t reg, uint8_t *value) {
  Adafruit_BusIO_Register bus_reg(_i2c_dev, reg);
  return bus_reg.read(value);
}

bool Adafruit_AS7331::readRegister(uint8_t reg, uint16_t *value) {
  Adafruit_BusIO_Register bus_reg(_i2c_dev, reg, 2, LSBFIRST);
  return bus_reg.read(value);
}

bool Adafruit_AS7331::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Adafruit_BusIO_Register bus_reg(_i2c_dev, reg, len, LSBFIRST);
  return bus_reg.read(buffer, len);
}
