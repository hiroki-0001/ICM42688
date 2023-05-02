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

#ifndef _FUSION_H
#define	_FUSION_H

#include "ICM42688Defs.hpp"

class Fusion
{
    public:
        Fusion();
        virtual ~Fusion();
        
        //  the following function can be called to set the SLERP power
        void setSlerpPower(float power) { m_slerpPower = power; }
        //  reset() resets the fusion state but keeps any setting changes (such as enables)
        virtual void reset() {}
        //  newIMUData() should be called for subsequent updates
        //  the fusion fields are updated with the results
        virtual void newIMUData(IMU_DATA& /* data */) {}
    
        void setGyroEnable(bool enable) { m_enableGyro = enable;}
        void setAccelEnable(bool enable) { m_enableAccel = enable; }

        inline const Vector3& getMeasuredPose() {return m_measuredPose;}
        inline const Quaternion& getMeasuredQPose() {return m_measuredQPose;}

        //  getAccelResiduals() gets the residual after subtracting gravity
        Vector3 getAccelResiduals();

    protected:
        void calculatePose(const Vector3& accel); // generates pose from accels
        Vector3 m_gyro;                                       // current gyro sample
        Vector3 m_accel;                                      // current accel sample

        Quaternion m_measuredQPose;       					// quaternion form of pose from measurement
        Vector3 m_measuredPose;								// vector form of pose from measurement
        Quaternion m_fusionQPose;                             // quaternion form of pose from fusion
        Vector3 m_fusionPose;                                 // vector form of pose from fusion

        Quaternion m_gravity;                                 // the gravity vector as a quaternion

        float m_slerpPower = 0;                                   // a value 0 to 1 that controls measured state influence
        Quaternion m_rotationDelta;                           // amount by which measured state differs from predicted
        Quaternion m_rotationPower;                           // delta raised to the appopriate power
        Vector3 m_rotationUnitVector;                         // the vector part of the rotation delta

        bool m_enableGyro;                                      // enables gyro as input
        bool m_enableAccel;                                     // enables accel as input

        bool m_firstTime_fusion = true;                                       // if first time after reset
        std::uint64_t m_lastFusionTime;                              // for delta time calculation
        static const char *m_fusionNameMap[];                   // the fusion name array
};

class FusionKalman4 : public Fusion
{
    public:
        FusionKalman4();
        ~FusionKalman4();
        //  reset() resets the kalman state but keeps any setting changes (such as enables)
        void reset();
        //  newIMUData() should be called for subsequent updates
        //  deltaTime is in units of seconds
        void newIMUData(IMU_DATA& data);
    
        void setQMatrix(Matrix4x4 Q) {  m_Q = Q; reset();}
        void setRkMatrix(Matrix4x4 Rk) { m_Rk = Rk; reset();}
    
    private:
        void predict();
        void update();
        
        Vector3 m_gyro;										// unbiased gyro data
        float m_timeDelta;                                    // time between predictions

        Quaternion m_stateQ;									// quaternion state vector
        Quaternion m_stateQError;                             // difference between stateQ and measuredQ

        Matrix4x4 m_Kk;                                       // the Kalman gain matrix
        Matrix4x4 m_Pkk_1;                                    // the predicted estimated covariance matrix
        Matrix4x4 m_Pkk;                                      // the updated estimated covariance matrix
        Matrix4x4 m_PDot;                                     // the derivative of the covariance matrix
        Matrix4x4 m_Q;                                        // process noise covariance
        Matrix4x4 m_Fk;                                       // the state transition matrix
        Matrix4x4 m_FkTranspose;                              // the state transition matrix transposed
        Matrix4x4 m_Rk;                                       // the measurement noise covariance
};

class FusionRTQF : public Fusion
{  
    public:
        FusionRTQF();
        ~FusionRTQF();
        //  reset() resets the state but keeps any setting changes (such as enables)
        void reset();
        //  newIMUData() should be called for subsequent updates
        //  deltaTime is in units of seconds
        void newIMUData(IMU_DATA& data);
    
    private:
        void predict();
        void update();
        Vector3 m_gyro;										// unbiased gyro data
        float m_timeDelta;                                    // time between predictions
        Quaternion m_stateQ;									// quaternion state vector
        Quaternion m_stateQError;                             // difference between stateQ and measuredQ
        int m_sampleNumber;

};

class madwick : public Fusion
{
    public:      
        madwick();
        void newIMUData(IMU_DATA& data);
        void reset(void);

    private:
        double q[4];					//! 推定したクオータニオンの値
        double dt;						//! サンプリングタイム(s)
        double gyroError;				//! 一秒間にジャイロセンサがずれる誤差(deg/s)
        double beta;					//! チューニングのための定数
        double roll;					//! ロールの値
        double pitch;					//! ピッチの値
        double yaw;						//! ヨーの値

};

#endif // _FUSION_H
