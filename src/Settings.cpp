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
        ERROR_LOG("Product name too long or null - using default");
        strcpy(m_filename, "ICM42688.ini");
    } else {
        sprintf(m_filename, "%s.ini", productType);
    }
    loadSettings();
}

Settings::Settings(const char *settingsDirectory, const char *productType)
{
    if (((strlen(productType) + strlen(settingsDirectory)) > 200) || (strlen(productType) == 0)) {
        ERROR_LOG("Product name too long or null - using default");
        strcpy(m_filename, "ICM42688.ini");
    } else {
        sprintf(m_filename, "%s/%s.ini", settingsDirectory, productType);
    }
    loadSettings();
}

void Settings::setDefaults()
{
    //  preset general defaults
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
        MESSAGE_LOG("Settings file not found. Using defaults and creating settings file")
        return saveSettings();
    }

    while (fgets(buf, 200, m_fd)) {
        if ((buf[0] == '#') || (buf[0] == ' ') || (buf[0] == '\n'))
            // just a comment
            continue;

        if (sscanf(buf, "%[^=]=%s", key, val) != 2) {
            ERROR_LOG1("Bad line in settings file: %s\n", buf)
            fclose(m_fd);
            return false;
        }

        // accel calibration
        if (strcmp(key, ACCELCAL_VALID) == 0) {
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
            ERROR_LOG1("Unrecognized key in settings file: %s\n", buf);
        }
    }
    MESSAGE_LOG1("Settings file %s loaded\n", m_filename);
    fclose(m_fd);
    return saveSettings();                                  // make sure settings file is correct and complete
}


bool Settings::saveSettings()
{
    if (!(m_fd = fopen(m_filename, "w"))) {
        ERROR_LOG("Failed to open settings file for save");
        return false;
    }

    //  General settings

    setComment("#####################################################################");
    setComment("");
    setComment("RTIMULib settings file");
    setBlank();
    setComment("General settings");
    setComment("");

    //  Accel calibration settings

    setBlank();
    setComment("#####################################################################");
    setComment("");

    setBlank();
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

    setBlank();
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