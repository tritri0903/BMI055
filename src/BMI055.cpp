#include "BMI055.h"
#include "Arduino.h"
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

    device->error_status = NO_ERROR;
    spi = new SPIClass(FSPI);
    spi->begin(device->PARAM_SPI_CLK, device->PARAM_SPI_PIN_MISO, device->PARAM_SPI_PIN_MOSI, device->PARAM_SPI_PIN_CS);

    pinMode(device->PARAM_SPI_PIN_CS, OUTPUT);
    pinMode(device->interuptPin, INPUT);
    

    if(readReg(device, ACC_CHIPID) == 0xFA){
        readReg(device, ACC_SFRSET, 0xb6);
        delay(500);
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
        delay(1000);
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
    temp_x = getX();
    temp_y = getY();
    temp_z = getZ();

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

void BMI055::calibrateDevice(int interval, deviceParam *device, int nbrPos){
    const uint8_t SAMPLE_LENGTH = 10;
    uint32_t startMillis = millis();
    uint32_t actualMillis = 0;
    uint16_t sampleRawLenght = 0, varianceLenght = 0;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t moy_x = 0, moy_y = 0, moy_z =0;
    uint32_t var_x = 0, var_y = 0, var_z = 0;
    uint64_t global_var[1500], global_var_sum, global_var_moy = 0;
    bool isMoving;

    Serial.println(millis());
    while (actualMillis < startMillis + interval)
    {
        actualMillis = millis();
        if(digitalRead(device->interuptPin))
        {
            device->rawPos.x[sampleRawLenght] = getX();
            device->rawPos.y[sampleRawLenght] = getY();
            device->rawPos.z[sampleRawLenght] = getZ();
            sampleRawLenght++;
            
        }
        if(sampleRawLenght>10000) break;
    }
    Serial.println(millis());
    Serial.print(sampleRawLenght);
    Serial.println(" samples -> Reading Finished -> Starting computation for mean");
    for(int j = 5; j < sampleRawLenght-10; j=j+5){
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;
        var_x = 0;
        var_y = 0;
        var_z = 0;
        for(int i = j - 5; i < j+SAMPLE_LENGTH - 5; i++){
            sum_x += device->rawPos.x[i];
            sum_y += device->rawPos.y[i];
            sum_z += device->rawPos.z[i];
        }
        moy_x = sum_x/SAMPLE_LENGTH;        
        moy_y = sum_y/SAMPLE_LENGTH;
        moy_z = sum_z/SAMPLE_LENGTH;
        for (int i = j - 5; i < j+SAMPLE_LENGTH - 5; i++)
        {
            var_x += sq(device->rawPos.x[i] - moy_x);
            var_y += sq(device->rawPos.y[i] - moy_y);
            var_z += sq(device->rawPos.z[i] - moy_z);
        }
        var_x /= SAMPLE_LENGTH;
        var_y /= SAMPLE_LENGTH;
        var_z /= SAMPLE_LENGTH;

        global_var[j/5] = sq(var_x) + sq(var_y) + sq(var_z);

        if(SAMPLE_LENGTH < j/5){
            for (int i = j/5 - SAMPLE_LENGTH; i < j/5; i++)
            {
                global_var_sum += global_var[i];
                varianceLenght = i;
            }
            global_var_moy = global_var_sum/10;
            global_var_sum = 0;            
        }

        if (global_var_moy > 500000000)
        {
            isMoving = true;
        }
        else isMoving = false;
        
        Serial.print(moy_x);
        Serial.print(",");
        Serial.print(moy_y);
        Serial.print(",");
        Serial.print(moy_z);
        Serial.print(",");
        Serial.print(global_var[j/5]);
        Serial.print(",");
        Serial.print(global_var_moy);
        Serial.print(",");
        Serial.println(isMoving);
    }
    

/*    for(int i = 0; i < nbrPos; i++){
        startMillis = millis();

        for(int i = 0; i < sampleRawLenght; i++){
            temp_x[i] = 0;
            temp_y[i] = 0;
            temp_z[i] = 0;
        }

        Serial.println(sum_x);
        Serial.println("Sum computed");
        for(int i = 0; i < sampleRawLenght; i++){
            //var_x += (temp_x[i] - (sum_x/sampleRawLenght)) * (temp_x[i] - (sum_x/sampleRawLenght));
            //var_y += (temp_y[i] - (sum_y/sampleRawLenght)) * (temp_y[i] - (sum_y/sampleRawLenght));
            //var_z += (temp_z[i] - (sum_z/sampleRawLenght)) * (temp_z[i] - (sum_z/sampleRawLenght));

            var_x += temp_x[i] * temp_x[i];
            var_y += temp_y[i] * temp_y[i];
            var_z += temp_z[i] * temp_z[i];
        }

        int32_t varx = var_x/sampleRawLenght - sum_x/sampleRawLenght;
        int32_t vary = var_y/sampleRawLenght - sum_y/sampleRawLenght;
        int32_t varz = var_z/sampleRawLenght - sum_z/sampleRawLenght;

        global_var = sqrt(varx*varx + vary*vary + varz*varz);
        

        Serial.print(var_x);
        Serial.print(", ");
        Serial.print(var_y);
        Serial.print(", ");
        Serial.print(var_z);

        Serial.print(" - ");
        Serial.println(global_var);    
        
        
        device->offsetPos[i].x = (int32_t)sum_x/sampleRawLenght;
        device->offsetPos[i].y = (int32_t)sum_y/sampleRawLenght;
        device->offsetPos[i].z = (int32_t)sum_z/sampleRawLenght;
        Serial.print(" x offset: ");
        Serial.println(device->offsetPos[i].x);
        Serial.println("Turn");
        delay(5000);
    }

    device->isCalibrated = true;*/

    
}

