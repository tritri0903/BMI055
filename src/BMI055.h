#ifndef _BMI055_H
#define _BMI055_H

#include "esp32-hal.h"

#define BMI055_ACC_ADDR     0x30
#define BMI055_GYRO_ADDR    0xd0

/********************ACC REGISTER VALUES *****************/
#define ACC_CHIPID          0x00
#define ACC_X_LSB           0x02
#define ACC_X_MSB           0x03
#define ACC_Y_LSB           0x04
#define ACC_Y_MSB           0x05
#define ACC_Z_LSB           0x06
#define ACC_Z_MSB           0x07
#define ACC_TEMP            0x08
#define ACC_RANGE           0x0f
#define ACC_BW              0x10
#define ACC_SFRSET          0x14
#define ACC_OFC_CTRL        0x36
#define ACC_OFC_OFFSET      0x37
#define ACC_OFFSET_X        0x38
#define ACC_OFFSET_Y        0x39
#define ACC_OFFSET_Z        0x3A
#define ACC_DATA_EN         0x17
#define ACC_INT1            0x1A

/*******************ACC CONFIGURATION VALUES *************/
#define ACC_RANGE_2G        0x03
#define ACC_RANGE_4G        0x04
#define ACC_RANGE_8G        0x08
#define ACC_RANGE_16G       0x0C

#define ACC_BW_7_81         0x08
#define ACC_BW_15_63        0x09
#define ACC_BW_31_25        0x0A
#define ACC_BW_62_5         0x0B
#define ACC_BW_125          0x0C
#define ACC_BW_250          0x0D
#define ACC_BW_500          0x0E
#define ACC_BW_1000         0x0F

#define ACC_OFC_CUTOFF_1HZ  0
#define ACC_OFC_CUTOFF_10HZ 1
#define ACC_OFC_AXE_X       32
#define ACC_OFC_AXE_Y       64
#define ACC_OFC_AXE_Z       96
#define ACC_OFC_SLOW_X      1
#define ACC_OFC_SLOW_Y      2
#define ACC_OFC_SLOW_Z      4

#define ACC_OFC_OFFSET_TARGET_X    1
#define ACC_OFC_OFFSET_TARGET_Y    3
#define ACC_OFC_OFFSET_TARGET_Z    5

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

    void setAccelerationFastCompensation(uint8_t offset_target_x, uint8_t offset_target_y, uint8_t offset_target_z);

    void getOffsets();
    void printOffset();

    void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
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

    uint8_t offsets[3];

    uint8_t readRegister(int reg);
    uint8_t writeRegister(int reg, int data);
  
    void setErrorStatus(Error value){
        error_status = value;
    }
  };

#endif
