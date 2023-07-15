#ifndef BMI055_h
#define BMI055_h

#include "Arduino.h"
#include "Wire.h"

class BMI055
{
  public:

    BMI055(); // object initializer 

    enum error{
      NO_ERROR,
      ERROR_NO_ANSWER,
      ERROR_WRONG_CHIP_ID,
      CONNECTED
    };

    struct deviceParam
    {
      uint8_t PARAM_SPI_CLK = -1;
      uint8_t PARAM_SPI_PIN_MOSI = -1;
      uint8_t PARAM_SPI_PIN_MISO = -1;
      uint8_t PARAM_SPI_PIN_CS = -1;
      uint32_t PARAM_SPI_CLK_FREQ = 100000;

      error error_status = NO_ERROR;

      uint8_t dataRx;

      bool isConnected;

      float x, y, z;
    } ;

    deviceParam accel;

    deviceParam gyro;

    void initialize(deviceParam *device); // sensor startup and various other functions - you need to run this line of code for the sensor to work

    void read_gyro(); // run this to read gyroscope data. gyroscopic data will be accessible in bno.gyro. x / y / z
    void read_accel(); // run this to read accelerometer data. accelerometer data will be accessible in bno.accel. x / y / z

    bool calibrate_gyro();
    void calibrate_accel();

    void avrg_reading();
    void accel_avrg_reading();

  private:
    uint8_t readReg(deviceParam *device, int reg, int data);
};

#endif
