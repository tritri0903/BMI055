#ifndef _BMI055_H
#define _BMI055_H

#include "Arduino.h"
#include <vector>

#define BMI055_ACC_ADDR     0x30
#define BMI055_GYRO_ADDR    0xd0

#define ACC_CHIPID          0x00
#define ACC_X_LSB           0x02
#define ACC_X_MSB           0x03
#define ACC_Y_LSB           0x04
#define ACC_Y_MSB           0x05
#define ACC_Z_LSB           0x06
#define ACC_Z_MSB           0x07
#define ACC_RANGE           0x0f
#define ACC_BW              0x10
#define ACC_SFRSET          0x14
#define ACC_OFC_CTRL        0x36
#define ACC_DATA_EN         0x17
#define ACC_INT1            0x1A

#define GYRO_CHIPID         0x00
#define GYRO_X_LSB          0x02
#define GYRO_X_MSB          0x03
#define GYRO_Y_LSB          0x04
#define GYRO_Y_MSB          0x05
#define GYRO_Z_LSB          0x06
#define GYRO_Z_MSB          0x07
#define GYRO_INT_STATUS1    0x0a
#define GYRO_RANGE          0x0f
#define GYRO_BW             0x10
#define GYRO_SFRSET         0x14
#define GYRO_INT_EN0        0x15
#define GYRO_SOC            0x31
#define GYRO_FOC            0x32
#define GYRO_INTMAP_1       0x1b

/************************************/
#define ACC_0G_X            2048
#define ACC_1G_X            (2048+1024)
#define ACC_MINUS1G_X       (2048-1024)
#define ACC_0G_Y            2048   
#define ACC_1G_Y            (2048+1024)
#define ACC_MINUS1G_Y       (2048-1024)
#define ACC_0G_Z            2048       
#define ACC_1G_Z            (2048+1024)
#define ACC_MINUS1G_Z       (2048-1024)
/****************************************/

using namespace std;

enum class Error {
    NoError,
    NoAnswer,
    WrongChipID,
    Connected
};

class BMI055 {
public:
    BMI055(); // object initializer 

    void beginSPI(uint8_t spiClk, uint8_t spiMosi, uint8_t spiMiso, uint8_t spiCs, uint32_t spiClkFreq = 100000);
    void beginI2C(uint8_t I2Cadd);

    int16_t getAccelerationX();
    int16_t getAccelerationY();
    int16_t getAccelerationZ();
    int16_t getRotationX();
    int16_t getRotationY();
    int16_t getRotationZ();

    void setInteruptPin(uint8_t value){
      INT_PIN = value;
      pinMode(INT_PIN, INPUT);
    }

    void setTotalCalibrationTime(uint16_t value) {
        TOTAL_CALIBRATION_TIME = value;
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

    uint8_t readRegister(int reg);
    uint8_t writeRegister(int reg, int data);
  
    void setErrorStatus(Error value){
        error_status = value;
    }
  };

#endif
