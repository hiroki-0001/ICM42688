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
//  The FusionKalman4 class
//
//----------------------------------------------------------

//  The QVALUE affects the gyro response.

#define KALMAN_QVALUE	0.001f

//  The RVALUE controls the influence of the accels and compass.
//  The bigger the value, the more sluggish the response.
#define KALMAN_RVALUE	0.0005f
#define KALMAN_QUATERNION_LENGTH	4
#define	KALMAN_STATE_LENGTH	4	// just the quaternion for the moment

FusionKalman4::FusionKalman4()
{
    std::cout << "fusion type = Kalman4" << std::endl;
    reset();
}

FusionKalman4::~FusionKalman4()
{
}

void FusionKalman4::reset()
{
    m_fusionPose = Vector3();
    m_fusionQPose.fromEuler(m_fusionPose);
    m_gyro = Vector3();
    m_accel = Vector3();
    m_measuredPose = Vector3();
    m_measuredQPose.fromEuler(m_measuredPose);
    m_Rk.fill(0);
    m_Q.fill(0);
    // initialize process noise covariance matrix
    for (int i = 0; i < KALMAN_STATE_LENGTH; i++)
        for (int j = 0; j < KALMAN_STATE_LENGTH; j++)
            m_Q.setVal(i, i, KALMAN_QVALUE);

    // initialize observation noise covariance matrix
    for (int i = 0; i < KALMAN_STATE_LENGTH; i++)
        for (int j = 0; j < KALMAN_STATE_LENGTH; j++)
            m_Rk.setVal(i, i, KALMAN_RVALUE);
}

void FusionKalman4::newIMUData(IMU_DATA& data)
{
    if (m_enableGyro)
        m_gyro = data.gyro;
    else
        m_gyro = Vector3();
    m_accel = data.accel;

    if (m_firstTime_fusion)
    {
        m_lastFusionTime = data.timestamp;
        calculatePose(m_accel);

        m_Fk.fill(0);
        //  init covariance matrix to something
        m_Pkk.fill(0);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m_Pkk.setVal(i,j, 0.5);

        // initialize the observation model Hk
        // Note: since the model is the state vector, this is an identity matrix so it won't be used

        //  initialize the poses
        m_stateQ.fromEuler(m_measuredPose);
        m_fusionQPose = m_stateQ;
        m_fusionPose = m_measuredPose;
        m_firstTime_fusion = false;
    }
    else
    {
        m_timeDelta = (float)(data.timestamp - m_lastFusionTime) / (float)1000000;
        m_lastFusionTime = data.timestamp;
        if (m_timeDelta <= 0)
            return;

        calculatePose(data.accel);
        predict();
        update();
        m_stateQ.toEuler(m_fusionPose);
        m_fusionQPose = m_stateQ;
        data.fusionPose = m_fusionPose;
        data.fusionQPose = m_fusionQPose;
    }

}

void FusionKalman4::predict()
{
    Matrix4x4 mat;
    Quaternion tQuat;
    float x2, y2, z2;

    //  compute the state transition matrix

    x2 = m_gyro.x() / (float)2.0;
    y2 = m_gyro.y() / (float)2.0;
    z2 = m_gyro.z() / (float)2.0;

    m_Fk.setVal(0, 1, -x2);
    m_Fk.setVal(0, 2, -y2);
    m_Fk.setVal(0, 3, -z2);

    m_Fk.setVal(1, 0, x2);
    m_Fk.setVal(1, 2, z2);
    m_Fk.setVal(1, 3, -y2);

    m_Fk.setVal(2, 0, y2);
    m_Fk.setVal(2, 1, -z2);
    m_Fk.setVal(2, 3, x2);

    m_Fk.setVal(3, 0, z2);
    m_Fk.setVal(3, 1, y2);
    m_Fk.setVal(3, 2, -x2);

    m_FkTranspose = m_Fk.transposed();

    // Predict new state estimate Xkk_1 = Fk * Xk_1k_1

    tQuat = m_Fk * m_stateQ;
    tQuat *= m_timeDelta;
    m_stateQ += tQuat;

    //    m_stateQ.normalize();

    // Compute PDot = Fk * Pk_1k_1 + Pk_1k_1 * FkTranspose (note Pkk == Pk_1k_1 at this stage)

    m_PDot = m_Fk * m_Pkk;
    mat = m_Pkk * m_FkTranspose;
    m_PDot += mat;

    // add in Q to get the new prediction

    m_Pkk_1 = m_PDot + m_Q;

    //  multiply by deltaTime (variable name is now misleading though)

    m_Pkk_1 *= m_timeDelta;
}

void FusionKalman4::update()
{
    Quaternion delta;
    Matrix4x4 Sk, SkInverse;

    if (m_enableAccel) {
        m_stateQError = m_measuredQPose - m_stateQ;
    } else {
        m_stateQError = Quaternion();
    }

    //	Compute residual covariance Sk = Hk * Pkk_1 * HkTranspose + Rk
    //  Note: since Hk is the identity matrix, this has been simplified

    Sk = m_Pkk_1 + m_Rk;

    //	Compute Kalman gain Kk = Pkk_1 * HkTranspose * SkInverse
    //  Note: again, the HkTranspose part is omitted

    SkInverse = Sk.inverted();

    m_Kk = m_Pkk_1 * SkInverse;

    // make new state estimate

    delta = m_Kk * m_stateQError;

    m_stateQ += delta;

    m_stateQ.normalize();

    //  produce new estimate covariance Pkk = (I - Kk * Hk) * Pkk_1
    //  Note: since Hk is the identity matrix, it is omitted

    m_Pkk.setToIdentity();
    m_Pkk -= m_Kk;
    m_Pkk = m_Pkk * m_Pkk_1;
}

//----------------------------------------------------------
//
//  The FusionRTQF class
//
//----------------------------------------------------------

FusionRTQF::FusionRTQF()
{
    std::cout << "fusion type = RTQF" << std::endl;
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
}

void FusionRTQF::update()
{
    if (m_enableAccel) {

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