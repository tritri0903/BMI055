#include "BMI055.h"
#include "Arduino.h"
#include "SPI.h"
#include <vector>


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

int16_t BMI055::getX(){
    if(CHIP_ID == 0xFA)
        return (int16_t)(writeRegister(ACC_X_LSB)|writeRegister(ACC_X_MSB)<<8);
    else
        return (int16_t)(readRegister(GYRO_X_LSB)|readRegister(GYRO_X_MSB)<<8);
};
int16_t BMI055::getY(){
    if(CHIP_ID == 0xFA)
        return (int16_t)(readRegister(ACC_Y_LSB)|readRegister(ACC_Y_MSB)<<8);
    else
        return (int16_t)(readRegister(GYRO_Y_LSB)|readRegister(GYRO_Y_MSB)<<8); 
};
int16_t BMI055::getZ(){
    if(CHIP_ID == 0xFA)
        return (int16_t)(readRegister(ACC_Z_LSB)|readRegister(ACC_Z_MSB)<<8);
    else
        return (int16_t)(readRegister(GYRO_Z_LSB)|readRegister(GYRO_Z_MSB)<<8);
};

int16_t BMI055::getXRotation(){
    return (int16_t)(readRegister(GYRO_X_LSB)|readRegister(GYRO_X_MSB)<<8);

};
int16_t BMI055::getYRotation(){
    return (int16_t)(readRegister(GYRO_Y_LSB)|readRegister(GYRO_Y_MSB)<<8); 

};
int16_t BMI055::getZRotation(){
    return (int16_t)(readRegister(GYRO_Z_LSB)|readRegister(GYRO_Z_MSB)<<8);
};

void BMI055::begin(uint8_t spiClk, uint8_t spiMosi, uint8_t spiMiso, uint8_t spiCs, uint32_t spiClkFreq) {
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
        writeRegister( ACC_SFRSET, 0xb6);
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

void BMI055::getDataset(vector<int16_t> &x, vector<int16_t> &y, vector<int16_t> &z, uint16_t &len){
    uint32_t startMillis = millis();
    uint32_t actualMillis = 0;

    len = 0;

    while (actualMillis < startMillis + TOTAL_CALIBRATION_TIME)
    {
        digitalWrite(LED_PIN, LOW);
        actualMillis = millis();
        if(digitalRead(INT_PIN))
        {
            try
            {
                x.at(len) = getX();
                y.at(len) = getY();
                z.at(len) = getZ();
                len++;
                digitalWrite(LED_PIN, HIGH);
            }
            catch(const std::exception& e)
            {
                Serial.println(e.what());
                x.resize(x.size() + 10, 0);
                y.resize(y.size() + 10, 0);
                z.resize(z.size() + 10, 0);
            }
        }
        if(len>5000) break;
    }
}

void BMI055::calibrateDevice(){
    const uint16_t SAMPLE_LENGTH = 5;
    uint32_t global_var_sum = 0;

    vector<int16_t> x(TOTAL_CALIBRATION_TIME/2, 0);
    vector<int16_t> y(TOTAL_CALIBRATION_TIME/2, 0);
    vector<int16_t> z(TOTAL_CALIBRATION_TIME/2, 0);
    uint16_t len;

    Serial.println(millis());
    getDataset(x, y, z, len);
    Serial.println(millis());
    Serial.print("Reading Finished: ");
    Serial.print(len);
    Serial.println(" samples");

    int16_t moy_x[x.size()/SAMPLE_LENGTH], moy_y[y.size()/SAMPLE_LENGTH], moy_z[z.size()/SAMPLE_LENGTH];
    vector<uint32_t> var_x(x.size()/SAMPLE_LENGTH), var_y(y.size()/SAMPLE_LENGTH), var_z(z.size()/SAMPLE_LENGTH);

    calculateAverage(x, len, moy_x, SAMPLE_LENGTH);
    calculateAverage(y, len, moy_y, SAMPLE_LENGTH);
    calculateAverage(z, len, moy_z, SAMPLE_LENGTH);

    calculateVariance(x, x.size(), moy_x, var_x, SAMPLE_LENGTH);
    calculateVariance(y, y.size(), moy_y, var_y, SAMPLE_LENGTH);
    calculateVariance(z, z.size(), moy_z, var_z, SAMPLE_LENGTH);

    vector<uint32_t> global_var(var_x.size(), 0);

    for (size_t i = 0; i < var_x.size(); i++)
    {
        global_var[i] = getGlobalVariance(var_x[i], var_y[i], var_z[i]);
    }

    vector<uint32_t> global_var_moy(global_var.size()/5 + 1, 0);

    calculateAverage(global_var, global_var.size(), global_var_moy, SAMPLE_LENGTH);

    for (size_t i = 0; i < len; i++)
    {
        Serial.print(x[i]);
        Serial.print(",");        
        Serial.print(moy_x[i/SAMPLE_LENGTH]);
        Serial.print(",");
        Serial.print(global_var[i/SAMPLE_LENGTH]);
        Serial.print(",");
        Serial.print(global_var_moy[i/25]);
        Serial.print(",");

        Serial.print(y[i]);
        Serial.print(",");
        Serial.print(moy_y[i/SAMPLE_LENGTH]);
        Serial.print(",");
        //Serial.print(var_y[i/SAMPLE_LENGTH]);
        //Serial.print(",");
        //
        Serial.print(z[i]);
        Serial.print(",");
        Serial.println(moy_z[i/SAMPLE_LENGTH]);
        //Serial.print(",");
        //Serial.println(var_z[i/SAMPLE_LENGTH]);
    }
    
    Serial.println("Finish fonction");
    
    /*
        global_var[j/5] = sq(var_x) + sq(var_y) + sq(var_z);
        if(j/5 > SAMPLE_LENGTH && j%5 == 0){
            global_var_sum = 0;
            uint16_t k = j/5;
            
            for (int i = k - 5; i < k + SAMPLE_LENGTH - 5; i++)
            {
                global_var_sum += global_var[i];
            }
            //global_var_moy = global_var_sum/10;
                      
        }

        if (global_var_moy[0] > 500000000)
        {
            isMoving = true;
            if(wasMoving == false){
                wasMoving = true;
                numberOfPos++;
            }
        }
        else {
            isMoving = false;
            wasMoving = false;
            if (moy_x < device->offsetPos[numberOfPos].low.x)
            {
                //Serial.println("recode offset");
                device->offsetPos[numberOfPos].low.x = moy_x;
            }
            if (moy_x > device->offsetPos[numberOfPos].high.x)
            {
                //Serial.println("recode offset");
                device->offsetPos[numberOfPos].high.x = moy_x;
            }
        }
        
        //Serial.print(moy_x);
        //Serial.print(",");
        //Serial.print(moy_y);
        //Serial.print(",");
        //Serial.print(global_var_sum);
        //Serial.print(",");
        //Serial.print(global_var[j/5]);
        //Serial.print(",");
        //Serial.print(global_var_moy[j/5]);
        //Serial.print(",");
        //Serial.println(varianceLenght);

    

    try
    {
        calculateAverage(global_var, 30, global_var_moy, 10);
    }
    catch(const char* errorMessage)
    {
        Serial.print("Error : " + String(errorMessage));
    }

    for (size_t i = 0; i < 1000 - 1; i++)
    {
        Serial.print(global_var[i]);
        Serial.print(",");
        Serial.println(global_var_moy[i/10]);
    }*/
    

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
            var_x += (temp_x[i] - (sum_x/sampleRawLenght)) * (temp_x[i] - (sum_x/sampleRawLenght));
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

void BMI055::calculateVariance(vector<int16_t> &valueArray, uint16_t valueLength,int16_t averages[], vector<uint32_t> &variances, uint8_t sampleSize){
    uint32_t variance = 0;
    uint16_t sampleIndex = 0;
    //Serial.println(valueArray.size());

    for (size_t i = 0; i < valueLength; i++)
    {
        variance += sq(valueArray[i] - averages[i/sampleSize]);

        if ((i + 1) % sampleSize == 0)
        {
            variances[sampleIndex] = variance / sampleSize;
            variance = 0;
            sampleIndex++;
        }
    }
    if (valueLength % sampleSize != 0)
    {
        variances[sampleIndex] = variance / (valueLength % sampleSize);
    }
    
    //Serial.println(sampleIndex);
}

uint32_t BMI055::getGlobalVariance(uint32_t varianceX, uint32_t varianceY, uint32_t varianceZ){
    return sqrt(sq(varianceX) + sq(varianceY) + sq(varianceZ));
}

void BMI055::calculateAverage(vector<int16_t> &dataArray, uint16_t dataLength, int16_t averages[], uint16_t sampleSize) {
    int32_t sum = 0;
    int sampleIndex = 0;

    for (size_t i = 0; i < dataLength; i++) {

        sum += dataArray[i];

        if ((i + 1) % sampleSize == 0) {  // Calculez la moyenne tous les échantillons de `sampleSize` valeurs
            averages[sampleIndex] = sum / sampleSize;
            sum = 0;  // Réinitialisez la somme pour le prochain échantillon
            sampleIndex++;
        }
    }

    // Si le nombre total de valeurs n'est pas un multiple de `sampleSize`,
    // calculez la moyenne des valeurs restantes
    if (dataLength % sampleSize != 0) {
        if ((dataLength % sampleSize) == 0) {
            throw "Division by zero";  // Lancez une exception si sampleSize est de zéro
        }
        averages[sampleIndex] = sum / (int16_t(dataLength) % sampleSize);
    }
    //Serial.println(sampleIndex);
}

void BMI055::calculateAverage(vector<uint32_t> &dataArray, uint16_t dataLength,vector<uint32_t> &averages, uint16_t sampleSize) {
    int32_t sum = 0;
    int sampleIndex = 0;

    for (size_t i = 0; i < dataLength; i++) {

        sum += dataArray[i];

        if ((i + 1) % sampleSize == 0) {  // Calculez la moyenne tous les échantillons de `sampleSize` valeurs
            averages[sampleIndex] = sum / sampleSize;
            sum = 0;  // Réinitialisez la somme pour le prochain échantillon
            sampleIndex++;
        }
    }

    // Si le nombre total de valeurs n'est pas un multiple de `sampleSize`,
    // calculez la moyenne des valeurs restantes
    if (dataLength % sampleSize != 0) {
        if ((dataLength % sampleSize) == 0) {
            throw "Division by zero";  // Lancez une exception si sampleSize est de zéro
        }
        averages[sampleIndex] = sum / (int16_t(dataLength) % sampleSize);
    }
    Serial.println(sampleIndex);
}

