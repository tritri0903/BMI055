#include "BMI055.h"
#include "Arduino.h"
#include "SPI.h"
#include <vector>

SPIClass *spi;

BMI055::BMI055(){};

uint8_t BMI055::readRegister(int reg) {
    
    uint8_t dataTxRx[2] = { (uint8_t) (reg | (1<<7)), (uint8_t) 0 };

    spi->beginTransaction(SPISettings(PARAM_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(PARAM_SPI_PIN_CS, LOW);

    spi->transfer(dataTxRx, 2);

    digitalWrite(PARAM_SPI_PIN_CS, HIGH);
    spi->endTransaction();

    return dataTxRx[1];
};

uint8_t BMI055::writeRegister(int reg, int data = 0) {
    
    uint8_t dataTxRx[2] = { (uint8_t) (reg | (1<<7)), (uint8_t) data };

    if(data){
        dataTxRx[0] = { (uint8_t) (reg | (0<<7))};
    }

    spi->beginTransaction(SPISettings(PARAM_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(PARAM_SPI_PIN_CS, LOW);

    spi->transfer(dataTxRx, 2);

    digitalWrite(PARAM_SPI_PIN_CS, HIGH);
    spi->endTransaction();

    return dataTxRx[1];
};

int16_t BMI055::getAccelerationX(){
        return (int16_t)(writeRegister(ACC_X_LSB)|writeRegister(ACC_X_MSB)<<8);
};
int16_t BMI055::getAccelerationY(){
        return (int16_t)(readRegister(ACC_Y_LSB)|readRegister(ACC_Y_MSB)<<8);
};
int16_t BMI055::getAccelerationZ(){
        return (int16_t)(readRegister(ACC_Z_LSB)|readRegister(ACC_Z_MSB)<<8);
};
int16_t BMI055::getRotationX(){
    return (int16_t)(readRegister(GYRO_X_LSB)|readRegister(GYRO_X_MSB)<<8);
};
int16_t BMI055::getRotationY(){
    return (int16_t)(readRegister(GYRO_Y_LSB)|readRegister(GYRO_Y_MSB)<<8); 
};
int16_t BMI055::getRotationZ(){
    return (int16_t)(readRegister(GYRO_Z_LSB)|readRegister(GYRO_Z_MSB)<<8);
};

void BMI055::beginSPI(uint8_t spiClk, uint8_t spiMosi, uint8_t spiMiso, uint8_t spiCs, uint32_t spiClkFreq) {
    PARAM_SPI_CLK = spiClk;
    PARAM_SPI_PIN_MOSI = spiMosi;
    PARAM_SPI_PIN_MISO = spiMiso;
    PARAM_SPI_PIN_CS = spiCs;
    setErrorStatus(Error::NoError);
    spi = new SPIClass(FSPI);
    spi->begin(spiClk, spiMiso, spiMosi, spiCs);

    pinMode(spiCs, OUTPUT);
    pinMode(INT_PIN, INPUT);

    CHIP_ID = readRegister(ACC_CHIPID);

    if(CHIP_ID == 0xFA){
        writeRegister(ACC_SFRSET, 0xb6);
        delay(500);
        writeRegister(ACC_RANGE, 0b0000011); // 2g
        delay(10);
        writeRegister(ACC_BW, 0x0d); // 500hz
        delay(10);
        writeRegister(ACC_DATA_EN, 0b00010000);
        delay(10);
        writeRegister(ACC_INT1, 0b00000001);
        error_status = Error::Connected;
    }
    else if (CHIP_ID == 0x0F){
        writeRegister(GYRO_SFRSET, 0xB6);
        delay(1000);
        writeRegister(GYRO_RANGE, 0x00); // range set to 2000°/s
        delay(10);
        writeRegister(GYRO_BW, 0b00000011); // 400Hz
        delay(10);
        writeRegister(GYRO_INT_EN0, 0x01); //new data int enable and auto-offset compensation
        delay(10);
        writeRegister(GYRO_INTMAP_1, 0b00000001);
        delay(10);
        error_status = Error::Connected;
    }
    else{
        error_status = Error::WrongChipID;
    }
    
};
