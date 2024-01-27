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
    
    uint8_t dataTxRx[2] = { (uint8_t) (reg | (0<<7)), (uint8_t) data };

    //if(data){
    //    dataTxRx[0] = { (uint8_t) (reg | (0<<7))};
    //}

    spi->beginTransaction(SPISettings(PARAM_SPI_CLK_FREQ, MSBFIRST, SPI_MODE0));
    digitalWrite(PARAM_SPI_PIN_CS, LOW);

    spi->transfer(dataTxRx, 2);

    digitalWrite(PARAM_SPI_PIN_CS, HIGH);
    spi->endTransaction();

    return dataTxRx[1];
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
        writeRegister(ACC_RANGE, ACC_RANGE_2G); // 2g
        delay(10);
        writeRegister(ACC_BW, ACC_BW_1000); // 
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


/** Set and activate the acceleration fast compensation mode. 
 * 
 * Each axe can be calibrate on a specifique target. Here is a table of the values:
 * 
 * <pre>
 *      TARGET      VALUES
 *      0           0g
 *      1           +1g
 *      2           -1g
 * </pre>
 * @param offset_target Target value for each axe
*/
void BMI055::setAccelerationFastCompensation(uint8_t offset_target_x, uint8_t offset_target_y, uint8_t offset_target_z){
    uint16_t cal_rdy;

    uint8_t offset_target = (offset_target_x<<ACC_OFC_OFFSET_TARGET_X)|(offset_target_y<<ACC_OFC_OFFSET_TARGET_Y)|(offset_target_z<<ACC_OFC_OFFSET_TARGET_Z);

    writeRegister(ACC_OFC_OFFSET, offset_target);

    writeRegister(ACC_OFC_CTRL, ACC_OFC_AXE_X);
    while(!cal_rdy){
        cal_rdy = readRegister(ACC_OFC_CTRL)&0x10;
        Serial.print(".");
    }
    cal_rdy = 0;

    writeRegister(ACC_OFC_CTRL, ACC_OFC_AXE_Y);
    while(!cal_rdy){
        cal_rdy = readRegister(ACC_OFC_CTRL)&0x10;
        Serial.print(".");
    }
    cal_rdy = 0;

    writeRegister(ACC_OFC_CTRL, ACC_OFC_AXE_Z);
    while(!cal_rdy){
        cal_rdy = readRegister(ACC_OFC_CTRL)&0x10;
        Serial.print(".");
    }
}

void BMI055::getOffsets(){
    offsets[0] = readRegister(ACC_OFFSET_X);
    offsets[1] = readRegister(ACC_OFFSET_Y);
    offsets[2] = readRegister(ACC_OFFSET_Z);
}

void BMI055::printOffset(){
    BMI055::getOffsets();
    for (size_t i = 0; i < 3; i++)
    {
        Serial.print(offsets[i]);
        Serial.print("-");
    }
    Serial.println("");
}

/** Get the X-axis acceleration value from the BMI055 accelerometer.
 *
 * @return 16-bit signed integer.
 */
int16_t BMI055::getAccelerationX(){
    return (int16_t)(readRegister(ACC_X_LSB)|readRegister(ACC_X_MSB)<<8);
};

/** Get the Y-axis acceleration value from the BMI055 accelerometer.
 *
 * @return 16-bit signed integer.
 */
int16_t BMI055::getAccelerationY(){
    return (int16_t)(readRegister(ACC_Y_LSB)|readRegister(ACC_Y_MSB)<<8);
};

/** Get the Z-axis acceleration value from the BMI055 accelerometer.
 *
 * @return 16-bit signed integer.
 */
int16_t BMI055::getAccelerationZ(){
    return (int16_t)(readRegister(ACC_Z_LSB)|readRegister(ACC_Z_MSB)<<8);
};

/** Get the X-axis rotation value from the BMI055 gyroscope.
 *
 * @return 16-bit signed integer.
 */
int16_t BMI055::getRotationX(){
    return (int16_t)(readRegister(GYRO_X_LSB)|readRegister(GYRO_X_MSB)<<8);
};

/** Get the Y-axis rotation value from the BMI055 gyroscope.
 *
 * @return 16-bit signed integer.
 */
int16_t BMI055::getRotationY(){
    return (int16_t)(readRegister(GYRO_Y_LSB)|readRegister(GYRO_Y_MSB)<<8); 
};

/** Get the Z-axis rotation value from the BMI055 gyroscope.
 *
 * @return 16-bit signed integer.
 */
int16_t BMI055::getRotationZ(){
    return (int16_t)(readRegister(GYRO_Z_LSB)|readRegister(GYRO_Z_MSB)<<8);
};

/** Get the acceleration values for all three axes (X, Y, Z) from the BMI055 accelerometer.
 *
 *  The results are returned as 16-bit signed integers through the pointers provided as arguments.
 *
 * @param ax Pointer to store the X-axis acceleration.
 * @param ay Pointer to store the Y-axis acceleration.
 * @param az Pointer to store the Z-axis acceleration.
 */
void BMI055::getAcceleration(int16_t* ax, int16_t* ay, int16_t* az){
    *ax = (int16_t)(BMI055::readRegister(ACC_X_LSB)|BMI055::readRegister(ACC_X_MSB)<<8);
    *ay = (int16_t)(BMI055::readRegister(ACC_Y_LSB)|BMI055::readRegister(ACC_Y_MSB)<<8);
    *az = (int16_t)(BMI055::readRegister(ACC_Z_LSB)|BMI055::readRegister(ACC_Z_MSB)<<8);
};