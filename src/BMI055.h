
#include "Arduino.h"

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

    struct offsetPosition
    {
        int x, y, z;
    };

    struct posValue
    {
      int16_t x[10000], y[10000], z[10000];
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

      offsetPosition offsetPos[4];
      posValue rawPos;

      bool isConnected;
      bool isCalibrated = false;
      bool posFinished = false;

      float x, y, z;

      float avg_x, avg_y, avg_z;

      uint8_t interuptPin = 5;
    } ;

    deviceParam accel;
    deviceParam gyro;

    void initialize(deviceParam *device); // sensor startup and various other functions - you need to run this line of code for the sensor to work

    void read_gyro(); // run this to read gyroscope data. gyroscopic data will be accessible in bno.gyro. x / y / z
    void getAccel(); // run this to read accelerometer data. accelerometer data will be accessible in bno.accel. x / y / z

    void getGyro();

    bool calibrate_gyro();
    void calibrateDevice(int interval, deviceParam *device,  int nbrPos);

    void avrg_reading();
    void accel_avrg_reading();

    bool getRawSample(int seconds, int nbr);

  private:
    int16_t getX();
    int16_t getY();
    int16_t getZ();
    int16_t getXRotation();
    int16_t getYRotation();
    int16_t getZRotation();
    uint8_t readReg(deviceParam *device, int reg, int data);
};
