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

#ifndef _SETTINGS_H
#define _SETTINGS_H

#include "IMUMath.hpp"
#include "HAL.hpp"

//  Settings keys
#define BUS_IS_I2C                 "BusIsI2C"
#define I2C_SLAVEADDRESS           "I2CSlaveAddress"
#define I2C_BUS                    "I2CBus"
#define SPI_BUS                    "SPIBus"
#define SPI_SELECT                 "SPISelect"
#define SPI_READ_SPEED             "SPIReadSpeed"
#define SPI_WRITE_SPEED            "SPIWriteSpeed"
#define AXIS_ROTATION              "AxisRotation"

//  ICM42688 settings keys
#define ICM42688_GYRO_ODR   "ICM42688GyroODR"
#define ICM42688_ACCEL_ODR  "ICM42688AccelODR"
#define ICM42688_GYRO_LPF   "ICM42688GyroLPF"
#define ICM42688_ACCEL_LPF  "ICM42688AccelLPF"
#define ICM42688_GYRO_FSR   "ICM42688GyroFSR"
#define ICM42688_ACCEL_FSR  "ICM42688AccelFSR"

//  Gyro bias keys

#define GYRO_BIAS_VALID            "GyroBiasValid"
#define GYRO_BIAS_X                "GyroBiasX"
#define GYRO_BIAS_Y                "GyroBiasY"
#define GYRO_BIAS_Z                "GyroBiasZ"

//  Accel calibration settings keys

#define ACCELCAL_VALID             "AccelCalValid"
#define ACCELCAL_MINX              "AccelCalMinX"
#define ACCELCAL_MAXX              "AccelCalMaxX"
#define ACCELCAL_MINY              "AccelCalMinY"
#define ACCELCAL_MAXY              "AccelCalMaxY"
#define ACCELCAL_MINZ              "AccelCalMinZ"
#define ACCELCAL_MAXZ              "AccelCalMaxZ"


class Settings : public HAL
{
public:

    //  Standard constructor sets up for ini file in working directory

    Settings(const char *productType = "ICM42688");

    //  Alternate constructor allow ini file to be in any directory

    Settings(const char *settingsDirectory, const char *productType);

    //  This function sets the settings to default values.

    void setDefaults();

    //  This function loads the local variables from the settings file or uses defaults

    virtual bool loadSettings();

    //  This function saves the local variables to the settings file

    virtual bool saveSettings();

    //  These are the local variables
    unsigned char m_I2CSlaveAddress;                        // I2C slave address of the imu
    int m_axisRotation;                                     // axis rotation code

    bool m_accelCalValid;                                   // true if there is valid accel calibration data
    Vector3 m_accelCalMin;                                // the minimum values
    Vector3 m_accelCalMax;                                // the maximum values

    bool m_gyroBiasValid;                                   // true if the recorded gyro bias is valid
    Vector3 m_gyroBias;                                   // the recorded gyro bias

    //  ICM42688
    uint8_t m_ICM42688GyroODR;
    uint8_t m_ICM42688AccelODR;
    uint8_t m_ICM42688GyroLPF;
    uint8_t m_ICM42688AccelLPF;
    uint8_t m_ICM42688GyroFSR;
    uint8_t m_ICM42688AccelFSR;

private:
    void setBlank();
    void setComment(const char *comment);
    void setValue(const char *key, const bool val);
    void setValue(const char *key, const int val);
    void setValue(const char *key, const float val);

    char m_filename[256];                                    // the settings file name

    FILE *m_fd;
};

#endif // _SETTINGS_H

