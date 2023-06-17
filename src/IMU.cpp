////////////////////////////////////////////////////////////////////////////
//  MIT License
//
//  Copyright (c) 2023 Hiroki, richards-tech, LLC
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

#include "IMU.hpp"
#include "ICM42688.hpp"

//  this defines the accelerometer noise level
#define RTIMU_FUZZY_GYRO_ZERO      0.20
//  this defines the accelerometer noise level
#define RTIMU_FUZZY_ACCEL_ZERO      0.05

//  Axis rotation arrays

float IMU::m_axisRotation[AXIS_ROTATION_COUNT][9] = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1},                    // RTIMU_XNORTH_YEAST
    {0, -1, 0, 1, 0, 0, 0, 0, 1},                   // RTIMU_XEAST_YSOUTH
    {-1, 0, 0, 0, -1, 0, 0, 0, 1},                  // RTIMU_XSOUTH_YWEST
    {0, 1, 0, -1, 0, 0, 0, 0, 1},                   // RTIMU_XWEST_YNORTH

    {1, 0, 0, 0, -1, 0, 0, 0, -1},                  // RTIMU_XNORTH_YWEST
    {0, 1, 0, 1, 0, 0, 0, 0, -1},                   // RTIMU_XEAST_YNORTH
    {-1, 0, 0, 0, 1, 0, 0, 0, -1},                  // RTIMU_XSOUTH_YEAST
    {0, -1, 0, -1, 0, 0, 0, 0, -1},                 // RTIMU_XWEST_YSOUTH

    {0, 1, 0, 0, 0, -1, -1, 0, 0},                  // RTIMU_XUP_YNORTH
    {0, 0, 1, 0, 1, 0, -1, 0, 0},                   // RTIMU_XUP_YEAST
    {0, -1, 0, 0, 0, 1, -1, 0, 0},                  // RTIMU_XUP_YSOUTH
    {0, 0, -1, 0, -1, 0, -1, 0, 0},                 // RTIMU_XUP_YWEST

    {0, 1, 0, 0, 0, 1, 1, 0, 0},                    // RTIMU_XDOWN_YNORTH
    {0, 0, -1, 0, 1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YEAST
    {0, -1, 0, 0, 0, -1, 1, 0, 0},                  // RTIMU_XDOWN_YSOUTH
    {0, 0, 1, 0, -1, 0, 1, 0, 0},                   // RTIMU_XDOWN_YWEST

    {1, 0, 0, 0, 0, 1, 0, -1, 0},                   // RTIMU_XNORTH_YUP
    {0, 0, -1, 1, 0, 0, 0, -1, 0},                  // RTIMU_XEAST_YUP
    {-1, 0, 0, 0, 0, -1, 0, -1, 0},                 // RTIMU_XSOUTH_YUP
    {0, 0, 1, -1, 0, 0, 0, -1, 0},                  // RTIMU_XWEST_YUP

    {1, 0, 0, 0, 0, -1, 0, 1, 0},                   // RTIMU_XNORTH_YDOWN
    {0, 0, 1, 1, 0, 0, 0, 1, 0},                    // RTIMU_XEAST_YDOWN
    {-1, 0, 0, 0, 0, 1, 0, 1, 0},                   // RTIMU_XSOUTH_YDOWN
    {0, 0, -1, -1, 0, 0, 0, 1, 0}                   // RTIMU_XWEST_YDOWN
};

IMU *IMU::createIMU(Settings *settings)
{
    return new ICM42688(settings);
}

IMU::IMU(Settings *settings)
{
    m_settings = settings;
    m_fusion = new FusionRTQF();
}

IMU::~IMU()
{
    delete m_fusion;
}

void IMU::convertToTemperature(unsigned char *rawData)
{
    setTemperature((rawData[0] / 132.48) + 25);
}

void IMU::updateFusion()
{
    m_fusion->newIMUData(m_imuData);
}

void IMU::gyroBiasInit()
{
    m_gyroLearningAlpha = 2.0f / m_sampleRate;
    m_gyroContinuousAlpha = 0.01f / m_sampleRate;
    m_gyroSampleCount = 0;
}


void IMU::handleGyroBias()
{


    if ((m_settings->m_axisRotation > 0) && (m_settings->m_axisRotation < AXIS_ROTATION_COUNT)) {
        // need to do an axis rotation
        float *matrix = m_axisRotation[m_settings->m_axisRotation];
        IMU_DATA tempIMU = m_imuData;

        // do new x value
        if (matrix[0] != 0) {
            m_imuData.gyro.setX(tempIMU.gyro.x() * matrix[0]);
            m_imuData.accel.setX(tempIMU.accel.x() * matrix[0]);
        } else if (matrix[1] != 0) {
            m_imuData.gyro.setX(tempIMU.gyro.y() * matrix[1]);
            m_imuData.accel.setX(tempIMU.accel.y() * matrix[1]);
        } else if (matrix[2] != 0) {
            m_imuData.gyro.setX(tempIMU.gyro.z() * matrix[2]);
            m_imuData.accel.setX(tempIMU.accel.z() * matrix[2]);
        }

        // do new y value
        if (matrix[3] != 0) {
            m_imuData.gyro.setY(tempIMU.gyro.x() * matrix[3]);
            m_imuData.accel.setY(tempIMU.accel.x() * matrix[3]);
        } else if (matrix[4] != 0) {
            m_imuData.gyro.setY(tempIMU.gyro.y() * matrix[4]);
            m_imuData.accel.setY(tempIMU.accel.y() * matrix[4]);
        } else if (matrix[5] != 0) {
            m_imuData.gyro.setY(tempIMU.gyro.z() * matrix[5]);
            m_imuData.accel.setY(tempIMU.accel.z() * matrix[5]);
        }

        // do new z value
        if (matrix[6] != 0) {
            m_imuData.gyro.setZ(tempIMU.gyro.x() * matrix[6]);
            m_imuData.accel.setZ(tempIMU.accel.x() * matrix[6]);
        } else if (matrix[7] != 0) {
            m_imuData.gyro.setZ(tempIMU.gyro.y() * matrix[7]);
            m_imuData.accel.setZ(tempIMU.accel.y() * matrix[7]);
        } else if (matrix[8] != 0) {
            m_imuData.gyro.setZ(tempIMU.gyro.z() * matrix[8]);
            m_imuData.accel.setZ(tempIMU.accel.z() * matrix[8]);
        }
    }

    Vector3 deltaAccel = m_previousAccel;
    deltaAccel -= m_imuData.accel;   // compute difference
    m_previousAccel = m_imuData.accel;

    if ((deltaAccel.length() < RTIMU_FUZZY_ACCEL_ZERO) && (m_imuData.gyro.length() < RTIMU_FUZZY_GYRO_ZERO)) {
        // what we are seeing on the gyros should be bias only so learn from this

        if (m_gyroSampleCount < (5 * m_sampleRate)) {
            m_settings->m_gyroBias.setX((1.0 - m_gyroLearningAlpha) * m_settings->m_gyroBias.x() + m_gyroLearningAlpha * m_imuData.gyro.x());
            m_settings->m_gyroBias.setY((1.0 - m_gyroLearningAlpha) * m_settings->m_gyroBias.y() + m_gyroLearningAlpha * m_imuData.gyro.y());
            m_settings->m_gyroBias.setZ((1.0 - m_gyroLearningAlpha) * m_settings->m_gyroBias.z() + m_gyroLearningAlpha * m_imuData.gyro.z());

            m_gyroSampleCount++;

            if (m_gyroSampleCount == (5 * m_sampleRate)) {
                // this could have been true already of course
                m_settings->m_gyroBiasValid = true;
                m_settings->saveSettings();
            }
        } else {
            m_settings->m_gyroBias.setX((1.0 - m_gyroContinuousAlpha) * m_settings->m_gyroBias.x() + m_gyroContinuousAlpha * m_imuData.gyro.x());
            m_settings->m_gyroBias.setY((1.0 - m_gyroContinuousAlpha) * m_settings->m_gyroBias.y() + m_gyroContinuousAlpha * m_imuData.gyro.y());
            m_settings->m_gyroBias.setZ((1.0 - m_gyroContinuousAlpha) * m_settings->m_gyroBias.z() + m_gyroContinuousAlpha * m_imuData.gyro.z());
        }
    }

    m_imuData.gyro -= m_settings->m_gyroBias;
}