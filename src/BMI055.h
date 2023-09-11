#ifndef MYCLASS_H
#define MYCLASS_H

#include "Arduino.h"
#include <vector>

using namespace std;

enum class Error {
    NoError,
    NoAnswer,
    WrongChipID,
    Connected
};

class Offset {
public:
    Offset(uint16_t x_val = 0, uint16_t y_val = 0, uint16_t z_val = 0)
        : x(x_val), y(y_val), z(z_val) {}

    uint16_t x;
    uint16_t y;
    uint16_t z;
};

class OffsetPosition {
public:
    OffsetPosition(const Offset& low_val, const Offset& high_val)
        : low(low_val), high(high_val) {}

    Offset low;
    Offset high;
};

class PosValue {
public:
    PosValue(size_t max_size = 1000)
        : x(max_size), y(max_size), z(max_size) {};

    vector<int16_t> x;
    vector<int16_t> y;
    vector<int16_t> z;

    uint16_t len;
};


class BMI055 {
public:
    BMI055(); // object initializer 

    void begin(uint8_t spiClk, uint8_t spiMosi, uint8_t spiMiso, uint8_t spiCs, uint32_t spiClkFreq = 100000);
    void begin(uint8_t I2Cadd);

    void calibrateDevice();

    void avrg_reading();
    void accel_avrg_reading();
    bool getRawSample(int seconds, int nbr);

    void setLedPin(uint8_t value) {
        LED_PIN = value;
        pinMode(LED_PIN, OUTPUT);
    }

    void setInteruptPin(uint8_t value){
      INT_PIN = value;
      pinMode(INT_PIN, INPUT);
    }

    void setTotalCalibrationTime(uint16_t value) {
        TOTAL_CALIBRATION_TIME = value;
    }

    void setErrorStatus(Error value){
        error_status = value;
    }

    Error getErrorStatus(){
        return error_status;
    }

private:
  uint8_t CHIP_ID;
  uint8_t PARAM_SPI_CLK;
  uint8_t PARAM_SPI_PIN_MOSI;
  uint8_t PARAM_SPI_PIN_MISO;
  uint8_t PARAM_SPI_PIN_CS;
  uint32_t PARAM_SPI_CLK_FREQ = 100000;

  uint8_t LED_PIN = -1;
  uint8_t INT_PIN = 5;
  Error error_status = Error::NoError;
  uint16_t TOTAL_CALIBRATION_TIME = 5000;
  std::vector<OffsetPosition> offsetPos;
  PosValue rawPos;

  bool isConnected = false;
  bool isCalibrated = false;
  bool posFinished = false;

  float avg_x = 0.0f;
  float avg_y = 0.0f;
  float avg_z = 0.0f;


  int16_t getX();
  int16_t getY();
  int16_t getZ();
  int16_t getXRotation();
  int16_t getYRotation();
  int16_t getZRotation();
  uint8_t readRegister(int reg);
  uint8_t writeRegister(int reg, int data);

  void getDataset(vector<int16_t> &x, vector<int16_t> &y, vector<int16_t> &z, uint16_t &len);

  void calculateAverage(vector<uint32_t> &dataArray, uint16_t dataLength,vector<uint32_t> &averages, uint16_t sampleSize);

  void calculateAverage(vector<int16_t> &dataArray, uint16_t dataLength, int16_t averages[], uint16_t sampleSize);

  void calculateVariance(vector<int16_t> &valueArray, uint16_t valueLength,int16_t averages[], vector<uint32_t> &variances, uint8_t sampleSize);
  
  uint32_t getGlobalVariance(uint32_t varianceX, uint32_t varianceY, uint32_t varianceZ);
};

#endif
