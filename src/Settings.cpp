////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The MPU-9250 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)


#include "Settings.hpp"
#include "ICM42688.hpp"


#define RATE_TIMER_INTERVAL 2

Settings::Settings(const char *productType)
{
    if ((strlen(productType) > 200) || (strlen(productType) == 0)) {
        HAL_ERROR("Product name too long or null - using default");
        strcpy(m_filename, "ICM42688.ini");
    } else {
        sprintf(m_filename, "%s.ini", productType);
    }
    loadSettings();
}

Settings::Settings(const char *settingsDirectory, const char *productType)
{
    if (((strlen(productType) + strlen(settingsDirectory)) > 200) || (strlen(productType) == 0)) {
        HAL_ERROR("Product name too long or null - using default");
        strcpy(m_filename, "ICM42688.ini");
    } else {
        sprintf(m_filename, "%s/%s.ini", settingsDirectory, productType);
    }
    loadSettings();
}

void Settings::setDefaults()
{
    //  preset general defaults
    m_I2CSlaveAddress = 104;
    m_busIsI2C = true;
    m_I2CBus = 1;
    m_SPIBus = 0;
    m_SPISelect = 0;
    m_SPIReadSpeed = 8000000;
    m_SPIWriteSpeed = 1000000;
    m_axisRotation = XNORTH_YEAST;
    m_accelCalValid = false;
    m_gyroBiasValid = false;

    //  MPU9250 defaults
    m_ICM42688GyroLPF   = LPF_1;
    m_ICM42688AccelLPF  = LPF_1;
    m_ICM42688GyroFSR   = GYRO_FSR_1000;
    m_ICM42688AccelFSR  = ACCEL_FSR_8 ;
    m_ICM42688GyroODR   = ODR_200;
    m_ICM42688AccelODR  = ODR_200;
}

bool Settings::loadSettings()
{
    setDefaults();
    char buf[200];
    char key[200];
    char val[200];
    float ftemp;
    //  check to see if settings file exists

    if (!(m_fd = fopen(m_filename, "r"))) {
        HAL_INFO("Settings file not found. Using defaults and creating settings file")
        return saveSettings();
    }

    while (fgets(buf, 200, m_fd)) {
        if ((buf[0] == '#') || (buf[0] == ' ') || (buf[0] == '\n'))
            // just a comment
            continue;

        if (sscanf(buf, "%[^=]=%s", key, val) != 2) {
            HAL_ERROR1("Bad line in settings file: %s\n", buf)
            fclose(m_fd);
            return false;
        }

        //  general config

        if (strcmp(key, BUS_IS_I2C) == 0) {
            m_busIsI2C = strcmp(val, "true") == 0;
        } else if (strcmp(key, I2C_BUS) == 0) {
            m_I2CBus = atoi(val);
        } else if (strcmp(key, SPI_BUS) == 0) {
            m_SPIBus = atoi(val);
        } else if (strcmp(key, SPI_SELECT) == 0) {
            m_SPISelect = atoi(val);
        } else if (strcmp(key, SPI_READ_SPEED) == 0) {
            m_SPIReadSpeed = atoi(val);
        } else if (strcmp(key, SPI_WRITE_SPEED) == 0) {
            m_SPIWriteSpeed = atoi(val);
        } else if (strcmp(key, I2C_SLAVEADDRESS) == 0) {
            m_I2CSlaveAddress = atoi(val);
        } else if (strcmp(key, AXIS_ROTATION) == 0) {
            m_axisRotation = atoi(val);

        // accel calibration

        }else if (strcmp(key, ACCELCAL_VALID) == 0) {
            m_accelCalValid = strcmp(val, "true") == 0;
        } else if (strcmp(key, ACCELCAL_MINX) == 0) {
            sscanf(val, "%f", &ftemp);
            m_accelCalMin.setX(ftemp);
        } else if (strcmp(key, ACCELCAL_MINY) == 0) {
            sscanf(val, "%f", &ftemp);
            m_accelCalMin.setY(ftemp);
        } else if (strcmp(key, ACCELCAL_MINZ) == 0) {
            sscanf(val, "%f", &ftemp);
            m_accelCalMin.setZ(ftemp);
        } else if (strcmp(key, ACCELCAL_MAXX) == 0) {
            sscanf(val, "%f", &ftemp);
            m_accelCalMax.setX(ftemp);
        } else if (strcmp(key, ACCELCAL_MAXY) == 0) {
            sscanf(val, "%f", &ftemp);
            m_accelCalMax.setY(ftemp);
        } else if (strcmp(key, ACCELCAL_MAXZ) == 0) {
            sscanf(val, "%f", &ftemp);
            m_accelCalMax.setZ(ftemp);

        // gyro bias
        
        } else if (strcmp(key, GYRO_BIAS_VALID) == 0) {
            m_gyroBiasValid = strcmp(val, "true") == 0;
        } else if (strcmp(key, GYRO_BIAS_X) == 0) {
            sscanf(val, "%f", &ftemp);
            m_gyroBias.setX(ftemp);
        } else if (strcmp(key, GYRO_BIAS_Y) == 0) {
            sscanf(val, "%f", &ftemp);
            m_gyroBias.setY(ftemp);
        } else if (strcmp(key, GYRO_BIAS_Z) == 0) {
            sscanf(val, "%f", &ftemp);
            m_gyroBias.setZ(ftemp);

        //  ICM42688 settings

        } else if (strcmp(key, ICM42688_GYRO_ODR) == 0) {
            m_ICM42688GyroODR = atoi(val);
        } else if (strcmp(key, ICM42688_ACCEL_ODR) == 0) {
            m_ICM42688AccelODR = atoi(val);
        } else if (strcmp(key, ICM42688_GYRO_LPF) == 0) {
            m_ICM42688GyroLPF = atoi(val);
        } else if (strcmp(key, ICM42688_ACCEL_LPF) == 0) {
            m_ICM42688AccelLPF = atoi(val);
        } else if (strcmp(key, ICM42688_GYRO_FSR) == 0) {
            m_ICM42688GyroFSR = atoi(val);
        } else if (strcmp(key, ICM42688_ACCEL_FSR) == 0) {
            m_ICM42688AccelFSR = atoi(val);

        //  Handle unrecognized key
        } else {
            HAL_ERROR1("Unrecognized key in settings file: %s\n", buf);
        }
    }
    HAL_INFO1("Settings file %s loaded\n", m_filename);
    fclose(m_fd);
    return saveSettings();                                  // make sure settings file is correct and complete
}


bool Settings::saveSettings()
{
    if (!(m_fd = fopen(m_filename, "w"))) {
        HAL_ERROR("Failed to open settings file for save");
        return false;
    }

    //  General settings

    setComment("#####################################################################");
    setComment("");
    setComment("IMU settings file");
    setComment("");


    setBlank();
    setComment("");
    setComment("Is bus I2C: 'true' for I2C, 'false' for SPI");
    setValue(BUS_IS_I2C, m_busIsI2C);

    setBlank();
    setComment("");
    setComment("I2C Bus (between 0 and 7) ");
    setValue(I2C_BUS, m_I2CBus);

    setBlank();
    setComment("");
    setComment("SPI Bus (between 0 and 7) ");
    setValue(SPI_BUS, m_SPIBus);

    setBlank();
    setComment("");
    setComment("SPI select (between 0 and 1) ");
    setValue(SPI_SELECT, m_SPISelect);

    setBlank();
    setComment("");
    setComment("SPI Read Speed in Hz");
    setValue(SPI_READ_SPEED, (int)m_SPIReadSpeed);

    setBlank();
    setComment("");
    setComment("SPI Write Speed in Hz");
    setValue(SPI_WRITE_SPEED, (int)m_SPIWriteSpeed);

    setBlank();
    setComment("");
    setComment("I2C slave address (filled in automatically by auto discover) ");
    setValue(I2C_SLAVEADDRESS, m_I2CSlaveAddress);

    setBlank();
    setComment("");
    
    setComment("Axis rotation");
    setBlank();
    setComment("These allow the IMU to be virtually repositioned if it is in a non-standard configuration ");
    setComment("Standard configuration is X pointing at north, Y pointing east and Z pointing down ");
    setComment("with the IMU horizontal. There are 24 different possible orientations as defined ");
    setComment("below. Setting the axis rotation code to non-zero values performs the repositioning.");

    setComment(" 0  - XNORTH_YEAST (default)    ");
    setComment(" 1  - XEAST_YSOUTH              ");
    setComment(" 2  - XSOUTH_YWEST              ");
    setComment(" 3  - XWEST_YNORTH              ");
    setComment(" 4  - XNORTH_YWEST              ");
    setComment(" 5  - XEAST_YNORTH              ");
    setComment(" 6  - XSOUTH_YEAST              ");
    setComment(" 7  - XWEST_YSOUTH              ");
    setComment(" 8  - XUP_YNORTH                ");
    setComment(" 9  - XUP_YEAST                 ");
    setComment(" 10 - XUP_YSOUTH                ");
    setComment(" 11 - XUP_YWEST                 ");
    setComment(" 12 - XDOWN_YNORTH              ");
    setComment(" 13 - XDOWN_YEAST               ");
    setComment(" 14 - XDOWN_YSOUTH              ");
    setComment(" 15 - XDOWN_YWEST               ");
    setComment(" 16 - XNORTH_YUP                ");
    setComment(" 17 - XEAST_YUP                 ");
    setComment(" 18 - XSOUTH_YUP                ");
    setComment(" 19 - XWEST_YUP                 ");
    setComment(" 20 - XNORTH_YDOWN              ");
    setComment(" 21 - XEAST_YDOWN               ");
    setComment(" 22 - XSOUTH_YDOWN              ");
    setComment(" 23 - XWEST_YDOWN               ");
    setValue(AXIS_ROTATION, m_axisRotation);

    //  Accel calibration settings

    setComment("#####################################################################");
    setComment("");
    setComment("Accel calibration");
    setValue(ACCELCAL_VALID, m_accelCalValid);
    setValue(ACCELCAL_MINX, m_accelCalMin.x());
    setValue(ACCELCAL_MINY, m_accelCalMin.y());
    setValue(ACCELCAL_MINZ, m_accelCalMin.z());
    setValue(ACCELCAL_MAXX, m_accelCalMax.x());
    setValue(ACCELCAL_MAXY, m_accelCalMax.y());
    setValue(ACCELCAL_MAXZ, m_accelCalMax.z());

    //  Gyro bias settings

    setBlank();
    setComment("#####################################################################");
    setComment("");
    setComment("Saved gyro bias data");
    setValue(GYRO_BIAS_VALID, m_gyroBiasValid);
    setValue(GYRO_BIAS_X, m_gyroBias.x());
    setValue(GYRO_BIAS_Y, m_gyroBias.y());
    setValue(GYRO_BIAS_Z, m_gyroBias.z());

    //  ICM42688 settings

    setBlank();
    setComment("#####################################################################");
    setComment("");
    setComment("ICM42688 settings");
    setComment("");
    setComment("#####################################################################");

    setBlank();
    setComment("");
    setComment("Gyro Output Data Rate - ");
    setComment("  1 - 32000Hz");
    setComment("  2 - 16000Hz");
    setComment("  3 - 8000Hz");
    setComment("  4 - 4000Hz");
    setComment("  5 - 2000Hz");
    setComment("  6 - 1000Hz (default)");
    setComment("  7 - 200Hz");
    setComment("  8 - 100Hz");
    setComment("  9 - 50Hz");
    setComment("  10 - 25Hz");
    setComment("  11 - 12.5Hz");
    setComment("  15 - 500Hz");
    setValue(ICM42688_GYRO_ODR, m_ICM42688GyroODR);

    setBlank();
    setComment("");
    setComment("Accel Output Data Rate - ");
    setComment("  1 - 32000Hz (LN mode only)");
    setComment("  2 - 16000Hz (LN mode only)");
    setComment("  3 - 8000Hz  (LN mode only)");
    setComment("  4 - 4000Hz  (LN mode only)");
    setComment("  5 - 2000Hz  (LN mode only)");
    setComment("  6 - 1000Hz  (LN mode only) (default)");
    setComment("  7 - 200Hz ");
    setComment("  8 - 100Hz");
    setComment("  9 - 50Hz");
    setComment("  10 - 25Hz");
    setComment("  11 - 12.5Hz");
    setComment("  12 - 6.25Hz (LP mode only)");
    setComment("  13 - 3.125Hz(LP mode only)");
    setComment("  14 - 1.5625Hz(LP mode only)");
    setComment("  15 - 500Hz");
    setValue(ICM42688_ACCEL_ODR, m_ICM42688AccelODR);

    setBlank();
    setComment("");
    setComment("Gyro low pass filter - ");
    setComment("  0 - ODR/2, (LN mode only)");
    setComment("  1 - LN mode : max(400Hz, ODR)/4, LP mode : 1x AVG filter (default)");
    setComment("  2 - max(400Hz, ODR)/5, (LN mode only)");
    setComment("  3 - max(400Hz, ODR)/8, (LN mode only)");
    setComment("  4 - max(400Hz, ODR)/10, (LN mode only)");
    setComment("  5 - max(400Hz, ODR)/16, (LN mode only)");
    setComment("  6 - max(400Hz, ODR)/20, (LN mode only)");
    setComment("  7 - max(400Hz, ODR)/40, (LN mode only)");
    setComment("  14 - Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)");
    setComment("  15 - Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)");
    setValue(ICM42688_GYRO_LPF, m_ICM42688GyroLPF);

    setBlank();
    setComment("");
    setComment("Accel low pass filter - ");
    setComment("  0 - ODR/2, (LN mode only)");
    setComment("  1 - LN mode : max(400Hz, ODR)/4, LP mode : 1x AVG filter (default)");
    setComment("  2 - max(400Hz, ODR)/5, (LN mode only)");
    setComment("  3 - max(400Hz, ODR)/8, (LN mode only)");
    setComment("  4 - max(400Hz, ODR)/10, (LN mode only)");
    setComment("  5 - max(400Hz, ODR)/16, (LN mode only)");
    setComment("  6 - max(400Hz, ODR)/20, (LN mode only)");
    setComment("  7 - max(400Hz, ODR)/40, (LN mode only)");
    setComment("  14 - Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)");
    setComment("  15 - Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)");
    setValue(ICM42688_ACCEL_LPF, m_ICM42688AccelLPF);

    setBlank();
    setComment("");
    setComment("Gyro full scale range - ");
    setComment("  0  - +/- 2000 degress per second");
    setComment("  1  - +/- 1000 degress per second");
    setComment("  2 - +/- 500 degress per second");
    setComment("  3 - +/- 250 degress per second");
    setComment("  4 - +/- 125 degress per second");
    setComment("  5 - +/- 62.5 degress per second");
    setComment("  6 - +/- 31.25 degress per second");
    setComment("  7 - +/- 15.625 degress per second");
    setValue(ICM42688_GYRO_FSR, m_ICM42688GyroFSR);

    setBlank();
    setComment("");
    setComment("Accel full scale range - ");
    setComment("  0  - +/- 16g");
    setComment("  1  - +/- 8g");
    setComment("  2 - +/- 4g");
    setComment("  3 - +/- 2g");
    setValue(ICM42688_ACCEL_FSR, m_ICM42688AccelFSR);

    fclose(m_fd);
    return true;
}

void Settings::setBlank()
{
    fprintf(m_fd, "\n");
}

void Settings::setComment(const char *comment)
{
    fprintf(m_fd, "# %s\n", comment);
}

void Settings::setValue(const char *key, const bool val)
{
    fprintf(m_fd, "%s=%s\n", key, val ? "true" : "false");
}

void Settings::setValue(const char *key, const int val)
{
    fprintf(m_fd, "%s=%d\n", key, val);
}

void Settings::setValue(const char *key, const float val)
{
    fprintf(m_fd, "%s=%f\n", key, val);
}