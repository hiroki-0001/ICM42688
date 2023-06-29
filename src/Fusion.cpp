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

#include "Fusion.hpp"
#include <iostream>

Fusion::Fusion()
{
    m_firstTime_fusion = true;
    m_enableGyro = true;
    m_enableAccel = true;

    m_gravity.setScalar(0);
    m_gravity.setX(0);
    m_gravity.setY(0);
    m_gravity.setZ(1);
}

Fusion::~Fusion()
{
}

Vector3 Fusion::getAccelResiduals()
{
    Quaternion rotatedGravity;
    Quaternion fusedConjugate;
    Quaternion qTemp;
    Vector3 residuals;

    //  do gravity rotation and subtraction
    // create the conjugate of the pose

    fusedConjugate = m_fusionQPose.conjugate();

    // now do the rotation - takes two steps with qTemp as the intermediate variable

    qTemp = m_gravity * m_fusionQPose;
    rotatedGravity = fusedConjugate * qTemp;

    // now adjust the measured accel and change the signs to make sense

    residuals.setX(-(m_accel.x() - rotatedGravity.x()));
    residuals.setY(-(m_accel.y() - rotatedGravity.y()));
    residuals.setZ(-(m_accel.z() - rotatedGravity.z()));
    return residuals;
}

void Fusion::calculatePose(const Vector3& accel)
{
    Quaternion m;
    Quaternion q;

    if (m_enableAccel) 
    {
        accel.accelToEuler(m_measuredPose);
    } 
    else 
    {
        m_measuredPose = m_fusionPose;
        m_measuredPose.setZ(0);
    }

    m_measuredPose.setZ(m_fusionPose.z());
    m_measuredQPose.fromEuler(m_measuredPose);

    //  check for quaternion aliasing. If the quaternion has the wrong sign
    //  the kalman filter will be very unhappy.

    int maxIndex = -1;
    float maxVal = -1000;
    
    for (int i = 0; i < 4; i++)
    {
        if (fabs(m_measuredQPose.data(i)) > maxVal)
        {
            maxVal = fabs(m_measuredQPose.data(i));
            maxIndex = i;
        }
    }

    if (((m_measuredQPose.data(maxIndex) < 0) && (m_fusionQPose.data(maxIndex) > 0)) ||
            ((m_measuredQPose.data(maxIndex) > 0) && (m_fusionQPose.data(maxIndex) < 0))) 
    {
        m_measuredQPose.setScalar(-m_measuredQPose.scalar());
        m_measuredQPose.setX(-m_measuredQPose.x());
        m_measuredQPose.setY(-m_measuredQPose.y());
        m_measuredQPose.setZ(-m_measuredQPose.z());
        m_measuredQPose.toEuler(m_measuredPose);
    }
}

//----------------------------------------------------------
//
//  The FusionRTQF class
//
//----------------------------------------------------------

FusionRTQF::FusionRTQF()
{
    reset();
}

FusionRTQF::~FusionRTQF()
{
}

void FusionRTQF::reset()
{
    m_firstTime_fusion = true;
    m_fusionPose = Vector3();
    m_fusionQPose.fromEuler(m_fusionPose);
    m_gyro = Vector3();
    m_accel = Vector3();
    m_measuredPose = Vector3();
    m_measuredQPose.fromEuler(m_measuredPose);
    m_sampleNumber = 0;
 }

 void FusionRTQF::newIMUData(IMU_DATA& data)
{
    m_sampleNumber++;

    if (m_enableGyro)
        m_gyro = data.gyro;
    else
        m_gyro = Vector3();

    m_accel = data.accel;

    if (m_firstTime_fusion) {
        m_lastFusionTime = data.timestamp;
        calculatePose(m_accel);

        //  initialize the poses

        m_stateQ.fromEuler(m_measuredPose);
        m_fusionQPose = m_stateQ;
        m_fusionPose = m_measuredPose;
        m_firstTime_fusion = false;
    } else {
        m_timeDelta = (float)(data.timestamp - m_lastFusionTime) / (float)1000000;
        m_lastFusionTime = data.timestamp;
        if (m_timeDelta <= 0)
            return;

        calculatePose(data.accel);

        predict();
        update();
        m_stateQ.toEuler(m_fusionPose);
        m_fusionQPose = m_stateQ;

    }
    data.fusionPose = m_fusionPose;
    data.fusionQPose = m_fusionQPose;
}

void FusionRTQF::predict()
{
    float x2, y2, z2;
    float qs, qx, qy,qz;

    if (!m_enableGyro)
        return;

    qs = m_stateQ.scalar();
    qx = m_stateQ.x();
    qy = m_stateQ.y();
    qz = m_stateQ.z();

    x2 = m_gyro.x() / (float)2.0;
    y2 = m_gyro.y() / (float)2.0;
    z2 = m_gyro.z() / (float)2.0;

    // Predict new state

    m_stateQ.setScalar(qs + (-x2 * qx - y2 * qy - z2 * qz) * m_timeDelta);
    m_stateQ.setX(qx + (x2 * qs + z2 * qy - y2 * qz) * m_timeDelta);
    m_stateQ.setY(qy + (y2 * qs - z2 * qx + x2 * qz) * m_timeDelta);
    m_stateQ.setZ(qz + (z2 * qs + y2 * qx - x2 * qy) * m_timeDelta);
    m_stateQ.normalize();

    const double x = m_gyro.x() * m_timeDelta;
    const double y = m_gyro.y() * m_timeDelta;
    const double z = m_gyro.z() * m_timeDelta;
    const double v = sqrt(x * x + y * y + z * z);
    if (v > 1e-6) {
        Quaternion dq(cos(v / 2), sin(v / 2) * x / v, sin(v / 2) * y / v, sin(v / 2) * z / v);
        Quaternion Q2 = Quaternion(qs,qx,qy,qz) * dq;
        //std::cout << "org: " << m_stateQ.scalar() << " " << m_stateQ.x() << std::endl;
        //std::cout << "myn: " << Q2.scalar() << " " << Q2.x() << std::endl;
        m_stateQ = Q2;
    }
    
}

void FusionRTQF::update()
{
    const float x = m_accel.x();
    const float y = m_accel.y();
    const float z = m_accel.z();
    const float norm = sqrt(x*x + y*y + z*z);
    const bool accel_valid = 0.95 < norm && norm < 1.05;
    const bool fall = fabs(x) > 0.5;

    if (accel_valid && !fall && m_enableAccel) 
    {
        // calculate rotation delta
        m_rotationDelta = m_stateQ.conjugate() * m_measuredQPose;
        m_rotationDelta.normalize();

        // take it to the power (0 to 1) to give the desired amount of correction

        float theta = acos(m_rotationDelta.scalar());

        float sinPowerTheta = sin(theta * m_slerpPower);
        float cosPowerTheta = cos(theta * m_slerpPower);

        m_rotationUnitVector.setX(m_rotationDelta.x());
        m_rotationUnitVector.setY(m_rotationDelta.y());
        m_rotationUnitVector.setZ(m_rotationDelta.z());
        m_rotationUnitVector.normalize();

        m_rotationPower.setScalar(cosPowerTheta);
        m_rotationPower.setX(sinPowerTheta * m_rotationUnitVector.x());
        m_rotationPower.setY(sinPowerTheta * m_rotationUnitVector.y());
        m_rotationPower.setZ(sinPowerTheta * m_rotationUnitVector.z());
        m_rotationPower.normalize();

        //  multiple this by predicted value to get result

        m_stateQ *= m_rotationPower;
        m_stateQ.normalize();
    }
}