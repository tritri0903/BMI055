#include "BMI055.h"
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "math.h"

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

int16_t BMI055::getX(){
    return (int16_t)(readReg(&accel, ACC_X_LSB)|readReg(&accel, ACC_X_MSB)<<8);

};
int16_t BMI055::getY(){
    return (int16_t)(readReg(&accel, ACC_Y_LSB)|readReg(&accel, ACC_Y_MSB)<<8);

};
int16_t BMI055::getZ(){
    return (int16_t)(readReg(&accel, ACC_Z_LSB)|readReg(&accel, ACC_Z_MSB)<<8);
};

int16_t BMI055::getXRotation(){
    return (int16_t)(readReg(&gyro, GYRO_X_LSB)|readReg(&gyro, GYRO_X_MSB)<<8);

};
int16_t BMI055::getYRotation(){
    return (int16_t)(readReg(&gyro, GYRO_Y_LSB)|readReg(&gyro, GYRO_Y_MSB)<<8); 

};
int16_t BMI055::getZRotation(){
    return (int16_t)(readReg(&gyro, GYRO_Z_LSB)|readReg(&gyro, GYRO_Z_MSB)<<8);
};

void BMI055::initialize(deviceParam *device) {
/**/ 
    device->error_status = NO_ERROR;
    spi = new SPIClass(FSPI);
    spi->begin(device->PARAM_SPI_CLK, device->PARAM_SPI_PIN_MISO, device->PARAM_SPI_PIN_MOSI, device->PARAM_SPI_PIN_CS);

    pinMode(device->PARAM_SPI_PIN_CS, OUTPUT);
    pinMode(device->interuptPin, INPUT);
    

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
        readReg(device, GYRO_SFRSET, 0xB6);
        delay(100);
        readReg(device, GYRO_RANGE, 0x00); // range set to 2000°/s
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
    readReg(&gyro, GYRO_SOC, B01000111);
    delay(10);
    //readReg(&gyro, GYRO_RANGE, 0x04);
    delay(10);
    readReg(&gyro, GYRO_FOC, B11000111);
    delay(10);
    readReg(&gyro, GYRO_FOC, B11101111);
    delay(100);
    readReg(&gyro, GYRO_RANGE, 0x00);

    while (readReg(&gyro, GYRO_FOC) == B00101111);

    return 1;
};

void BMI055::getGyro() {
    int16_t temp_x, temp_y, temp_z;

    //Reading angular rate 
    temp_x = getXRotation();
    temp_y = getYRotation(); 
    temp_z = getZRotation();

    
    gyro.x = temp_x / 16.4f; //(sum_x / 20) ;
    gyro.y = temp_y / 16.4f; //(sum_y / 20) ;
    gyro.z = temp_z / 16.4f; //(sum_z / 20) ;
    
};

void BMI055::getAccel() {
    int16_t temp_x, temp_y, temp_z;

    //Reading  
    temp_x = (int16_t)(readReg(&accel, ACC_X_LSB)|readReg(&accel, ACC_X_MSB)<<8);
    temp_y = (int16_t)(readReg(&accel, ACC_Y_LSB)|readReg(&accel, ACC_Y_MSB)<<8);
    temp_z = (int16_t)(readReg(&accel, ACC_Z_LSB)|readReg(&accel, ACC_Z_MSB)<<8);

    accel.x = temp_x ;//* 0.0653f ;
    accel.y = temp_y ;//* 0.0653f ;
    accel.z = temp_z ;//* 0.0653f ;
    
};

bool BMI055::getRawSample(int seconds, int nbr){
    int16_t temp_x[nbr], temp_y[nbr], temp_z[nbr];
    int sum_x, sum_y, sum_z;

    for (size_t i = 0; i < nbr; i++)
    {
        while (!digitalRead(accel.interuptPin));
        
        sum_x += (int16_t)(readReg(&accel, ACC_X_LSB)|readReg(&accel, ACC_X_MSB)<<8);
        sum_y += (int16_t)(readReg(&accel, ACC_Y_LSB)|readReg(&accel, ACC_Y_MSB)<<8);
        sum_z += (int16_t)(readReg(&accel, ACC_Z_LSB)|readReg(&accel, ACC_Z_MSB)<<8);

        delay(10);
    }

    accel.avg_x = sum_x/nbr;
    accel.avg_y = sum_y/nbr;
    accel.avg_z = sum_z/nbr;

    return 1;

}

void BMI055::calibrateDevice(int interval, deviceParam *device){
    uint32_t startMillis;
    uint32_t actualMillis;
    uint16_t count;
    
    for(int i = 0; i <= 3; i++){
        startMillis = millis();
        int16_t temp_x[200],  temp_y[200], temp_z[200];
        int16_t sum_x = 0, sum_y = 0, sum_z = 0;
        int32_t var_x = 0, var_y = 0, var_z = 0;
        int64_t global_var = 0;
        count = 0;
        while (actualMillis < startMillis + interval)
        {
            actualMillis = millis();
            if (digitalRead(device->interuptPin))
            {
                temp_x[count] = (int16_t)(readReg(&accel, ACC_X_LSB)|readReg(&accel, ACC_X_MSB)<<8);
                temp_y[count] = (int16_t)(readReg(&accel, ACC_Y_LSB)|readReg(&accel, ACC_Y_MSB)<<8);
                temp_z[count] = (int16_t)(readReg(&accel, ACC_Z_LSB)|readReg(&accel, ACC_Z_MSB)<<8);
                delay(10);
                count++;
                Serial.print(".");
            }
        }
        Serial.print(i);
        Serial.print(" N  Reading Finished ->");
        Serial.println(count);
        for(int i = 0; i < count; i++){
            sum_x += temp_x[i];
            sum_y += temp_y[i];
            sum_z += temp_z[i];
        }
        Serial.println("Sum computed");
        for(int i = 0; i < count; i++){
            //var_x += (temp_x[i] - (sum_x/count)) * (temp_x[i] - (sum_x/count));
            //var_y += (temp_y[i] - (sum_y/count)) * (temp_y[i] - (sum_y/count));
            //var_z += (temp_z[i] - (sum_z/count)) * (temp_z[i] - (sum_z/count));

            var_x += temp_x[i] * temp_x[i];
            var_y += temp_y[i] * temp_y[i];
            var_z += temp_z[i] * temp_z[i];
        }

        var_x = var_x/count - sum_x/count;
        var_y = var_y/count - sum_y/count;
        var_z = var_z/count - sum_z/count;

        global_var = sqrt(var_x*var_x + var_y*var_y + var_z*var_z);
        

        Serial.print(var_x);
        Serial.print(", ");
        Serial.print(var_y);
        Serial.print(", ");
        Serial.print(var_z);

        Serial.print(" - ");
        Serial.println(global_var);    
        
        
        device->offsetPos[i].x = (int32_t)sum_x/count;
        device->offsetPos[i].y = (int32_t)sum_y/count;
        device->offsetPos[i].z = (int32_t)sum_z/count;
        Serial.print(" x offset: ");
        Serial.println(device->offsetPos[i].x);
        Serial.println("Turn");
        delay(5000);
    }

    device->isCalibrated = true;
    
}

