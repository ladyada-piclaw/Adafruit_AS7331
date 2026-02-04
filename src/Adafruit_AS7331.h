#pragma once

#include <Adafruit_BusIO_Register.h>
#include <Arduino.h>
#include <Wire.h>

#define AS7331_DEFAULT_ADDRESS 0x74

#define AS7331_REG_OSR 0x00
#define AS7331_REG_STATUS                                                      \
  0x00 // STATUS is byte 1 when reading 16-bits from OSR in measurement state
#define AS7331_REG_TEMP 0x01
#define AS7331_REG_AGEN 0x02
#define AS7331_REG_MRES1 0x02
#define AS7331_REG_MRES2 0x03
#define AS7331_REG_MRES3 0x04
#define AS7331_REG_CREG1 0x06
#define AS7331_REG_CREG2 0x07
#define AS7331_REG_CREG3 0x08
#define AS7331_REG_BREAK 0x09
#define AS7331_REG_EDGES 0x0A

#define AS7331_STATUS_OUTCONVOF (1 << 7) // Output conversion overflow
#define AS7331_STATUS_MRESOF (1 << 6)    // Measurement result overflow
#define AS7331_STATUS_ADCOF (1 << 5)     // ADC overflow
#define AS7331_STATUS_LDATA (1 << 4)     // Data in OUTCONV registers
#define AS7331_STATUS_NDATA (1 << 3)     // New data available
#define AS7331_STATUS_NOTREADY (1 << 2)  // Not ready (conversion in progress)

#define AS7331_PART_ID 0x21

typedef enum {
  AS7331_GAIN_2048X = 0,
  AS7331_GAIN_1024X = 1,
  AS7331_GAIN_512X = 2,
  AS7331_GAIN_256X = 3,
  AS7331_GAIN_128X = 4,
  AS7331_GAIN_64X = 5,
  AS7331_GAIN_32X = 6,
  AS7331_GAIN_16X = 7,
  AS7331_GAIN_8X = 8,
  AS7331_GAIN_4X = 9,
  AS7331_GAIN_2X = 10,
  AS7331_GAIN_1X = 11,
} as7331_gain_t;

typedef enum {
  AS7331_TIME_1MS = 0,
  AS7331_TIME_2MS = 1,
  AS7331_TIME_4MS = 2,
  AS7331_TIME_8MS = 3,
  AS7331_TIME_16MS = 4,
  AS7331_TIME_32MS = 5,
  AS7331_TIME_64MS = 6,
  AS7331_TIME_128MS = 7,
  AS7331_TIME_256MS = 8,
  AS7331_TIME_512MS = 9,
  AS7331_TIME_1024MS = 10,
  AS7331_TIME_2048MS = 11,
  AS7331_TIME_4096MS = 12,
  AS7331_TIME_8192MS = 13,
  AS7331_TIME_16384MS = 14,
} as7331_time_t;

typedef enum {
  AS7331_MODE_CONT = 0,
  AS7331_MODE_CMD = 1,
  AS7331_MODE_SYNS = 2,
  AS7331_MODE_SYND = 3,
} as7331_mode_t;

typedef enum {
  AS7331_CLOCK_1024MHZ = 0,
  AS7331_CLOCK_2048MHZ = 1,
  AS7331_CLOCK_4096MHZ = 2,
  AS7331_CLOCK_8192MHZ = 3,
} as7331_clock_t;

class Adafruit_AS7331 {
public:
  Adafruit_AS7331();

  bool begin(TwoWire *wire = &Wire, uint8_t addr = AS7331_DEFAULT_ADDRESS);

  bool powerDown(bool pd);
  bool setMeasurementMode(as7331_mode_t mode);
  as7331_mode_t getMeasurementMode(void); // Get current measurement mode

  bool reset(void);          // Software reset
  uint8_t getDeviceID(void); // Returns part ID (expect 0x21)

  bool setGain(as7331_gain_t gain);
  as7331_gain_t getGain(void);

  bool setIntegrationTime(as7331_time_t time);
  as7331_time_t getIntegrationTime(void);

  bool setClockFrequency(as7331_clock_t clock);
  as7331_clock_t getClockFrequency(void);

  uint16_t readUVA(void);
  uint16_t readUVB(void);
  uint16_t readUVC(void);
  bool readAllUV(uint16_t *uva, uint16_t *uvb, uint16_t *uvc);

  float readUVA_uWcm2(void); // Read UVA and convert to µW/cm²
  float readUVB_uWcm2(void); // Read UVB and convert to µW/cm²
  float readUVC_uWcm2(void); // Read UVC and convert to µW/cm²
  bool readAllUV_uWcm2(float *uva, float *uvb, float *uvc);

  bool oneShot(uint16_t *uva, uint16_t *uvb,
               uint16_t *uvc); // Single measurement in CMD mode
  bool oneShot_uWcm2(float *uva, float *uvb,
                     float *uvc); // Single measurement with µW/cm² conversion

  float readTemperature(void);
  bool isDataReady(void);
  uint8_t getStatus(void);
  bool hasOverflow(void);
  bool hasNewData(void);

  bool setReadyPinOpenDrain(bool openDrain); // true=open-drain, false=push-pull
  bool getReadyPinOpenDrain(void);

  bool setBreakTime(uint8_t breakTime); // 0-255, time = breakTime * 8µs
  uint8_t getBreakTime(void);

  bool setEdgeCount(uint8_t edges); // 1-255 for SYND mode, 0 treated as 1
  uint8_t getEdgeCount(void);
  bool startMeasurement(void); // Set SS=1
  bool stopMeasurement(void);  // Set SS=0
  bool hasLostData(void);      // STATUS:LDATA flag

  bool enableDivider(bool enable);
  bool setDivider(uint8_t div); // 0-7, factor = 2^(1+div)
  uint8_t getDivider(void);

  bool setStandby(bool enable);
  bool getStandby(void);

private:
  bool readRegister(uint8_t reg, uint8_t *value);
  bool readRegister(uint8_t reg, uint16_t *value);
  bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);

  float _countsToIrradiance(uint16_t counts, float baseSensitivity);

  Adafruit_I2CDevice *_i2c_dev = nullptr;
  uint8_t _cached_gain = 10; // Default: AS7331_GAIN_2X
  uint8_t _cached_time = 6;  // Default: AS7331_TIME_64MS
};
