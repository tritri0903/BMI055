#include "BMI055.h"
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#define ACC_CHIPID  0x00
#define ACC_X_LSB   0x02
#define ACC_X_MSB   0x03
#define ACC_Y_LSB   0x04
#define ACC_Y_MSB   0x05
#define ACC_Z_LSB   0x06
#define ACC_Z_MSB   0x07
#define ACC_RANGE   0x0f
#define ACC_BW      0x10
#define ACC_SFRSET  0x14
#define ACC_OFC_CTRL     0x36
#define ACC_DATA_EN 0x17
#define ACC_INT1    0x1A

#define GYRO_CHIPID       0x00
#define GYRO_X_LSB        0x02
#define GYRO_X_MSB        0x03
#define GYRO_Y_LSB        0x04
#define GYRO_Y_MSB        0x05
#define GYRO_Z_LSB        0x06
#define GYRO_Z_MSB        0x07
#define GYRO_INT_STATUS1  0x0a
#define GYRO_RANGE        0x0f
#define GYRO_BW           0x10
#define GYRO_SFRSET       0x14
#define GYRO_INT_EN0      0x15
#define GYRO_SOC          0x31
#define GYRO_FOC          0x32
#define GYRO_INTMAP_1     0x1b

#define BMI055_ACC_ADDR  0x30
#define BMI055_GYRO_ADDR 0xd0
/************************************/
#define ACC_0G_X      2048
#define ACC_1G_X      (2048+1024)
#define ACC_MINUS1G_X (2048-1024)
#define ACC_0G_Y      2048   
#define ACC_1G_Y      (2048+1024)
#define ACC_MINUS1G_Y (2048-1024)
#define ACC_0G_Z      2048       
#define ACC_1G_Z      (2048+1024)
#define ACC_MINUS1G_Z (2048-1024)
/****************************************/

SPIClass *spi;

BMI055::BMI055(){};

uint8_t BMI055::readReg(deviceParam *device, int reg, int data = 0) {
    
    uint8_t dataTxRx[2] = { (uint8_t) (reg | (1<<7)), (uint8_t) data };

    if(data){
        dataTxRx[0] = { (uint8_t) (reg | (0<<7))};
    }

    spi->beginTransaction(SPISettings(device->PARAM_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(device->PARAM_SPI_PIN_CS, LOW);

    spi->transfer(dataTxRx, 2);

    digitalWrite(device->PARAM_SPI_PIN_CS, HIGH);
    spi->endTransaction();

    return dataTxRx[1];
};

void BMI055::initialize(deviceParam *device) {
/**/ 
    device->error_status = NO_ERROR;
    spi = new SPIClass(FSPI);
    spi->begin(device->PARAM_SPI_CLK, device->PARAM_SPI_PIN_MISO, device->PARAM_SPI_PIN_MOSI, device->PARAM_SPI_PIN_CS);

    pinMode(device->PARAM_SPI_PIN_CS, OUTPUT);

    if(readReg(device, ACC_CHIPID) == 0xFA){
        //readReg(device, ACC_SFRSET, 0xb6);
        delay(100);
        readReg(device, ACC_RANGE, 0b0000011); // 2g
        delay(10);
        readReg(device, ACC_BW, 0x0d); // 500hz
        delay(10);
        readReg(device, ACC_DATA_EN, 0b00010000);
        delay(10);
        readReg(device, ACC_INT1, 0b00000001);
        device->error_status = CONNECTED;
    }
    else if (readReg(device, GYRO_CHIPID) == 0x0F){
        readReg(device, GYRO_RANGE, 0x01);
        delay(10);
        readReg(device, GYRO_BW, 0b00000011); // 400Hz
        delay(10);
        readReg(device, GYRO_INT_EN0, 0x01); //new data int enable and auto-offset compensation
        delay(10);
        readReg(device, GYRO_INTMAP_1, 0b00000001);
        delay(10);
        device->error_status = CONNECTED;
    }
    else{
        device->error_status = ERROR_WRONG_CHIP_ID;
    }
    
};

bool BMI055::calibrate_gyro(){

    readReg(&gyro, GYRO_FOC, B00000111);
    delay(10);
    readReg(&gyro, GYRO_FOC, B00101111);
    delay(10);

    while (readReg(&gyro, GYRO_FOC, 0) == B00000111)
    {
        /* code */
    }
    return 0;
};

void BMI055::read_gyro() {

    gyro.x = (int16_t)(readReg(&gyro, GYRO_X_LSB)|readReg(&gyro, GYRO_X_MSB)<<8) * 0.03051757812; // same as x / 32768.0 * 1000.0
    gyro.y = (int16_t)(readReg(&gyro, GYRO_Y_LSB)|readReg(&gyro, GYRO_Y_MSB)<<8) * 0.03051757812; 
    gyro.z = (int16_t)(readReg(&gyro, GYRO_Z_LSB)|readReg(&gyro, GYRO_Z_MSB)<<8) * 0.03051757812; 

    avrg_reading();
    
};

void BMI055::avrg_reading(){
    float sumx = 0;
    float sumy = 0;
    float sumz = 0;
    for (int i = 0; i < 20; i++){
        sumx += gyro.x;
        sumy += gyro.y;
        sumz += gyro.z;
        delay(10);
    }

    gyro.x = sumx/20;
    gyro.y = sumy/20;
    gyro.z = sumz/20;
};

void BMI055::read_accel() {
    
    accel.x = (int16_t)(readReg(&accel, ACC_X_LSB)|readReg(&accel, ACC_X_MSB)<<8);
    accel.y = (int16_t)(readReg(&accel, ACC_Y_LSB)|readReg(&accel, ACC_Y_MSB)<<8);
    accel.z = (int16_t)(readReg(&accel, ACC_Z_LSB)|readReg(&accel, ACC_Z_MSB)<<8);
    accel.x /= 16384;
    accel.y /= 16384;
    accel.z /= 16384;

    //accel_avrg_reading();
};

void BMI055::accel_avrg_reading(){
    float sumx = 0;
    float sumy = 0;
    float sumz = 0;
    for (int i = 0; i < 20; i++){
        sumx += accel.x;
        sumy += accel.y;
        sumz += accel.z;
        delay(10);
    }

    accel.x = sumx/20;
    accel.y = sumy/20;
    accel.z = sumz/20;
};
