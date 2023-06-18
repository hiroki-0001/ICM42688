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

#ifndef _IMU_H
#define	_IMU_H

#include "Fusion.hpp"
#include "Settings.hpp"

//  Axis rotation defs
//
//  These allow the IMU to be virtually repositioned if it is in a non-standard configuration
//  Standard configuration is X pointing at north, Y pointing east and Z pointing down
//  with the IMU horizontal. There are 24 different possible orientations as defined
//  below. Setting the axis rotation code to non-zero values performs the repositioning.

#define XNORTH_YEAST              0                   // this is the default identity matrix
#define XEAST_YSOUTH              1
#define XSOUTH_YWEST              2
#define XWEST_YNORTH              3
#define XNORTH_YWEST              4
#define XEAST_YNORTH              5
#define XSOUTH_YEAST              6
#define XWEST_YSOUTH              7
#define XUP_YNORTH                8
#define XUP_YEAST                 9
#define XUP_YSOUTH                10
#define XUP_YWEST                 11
#define XDOWN_YNORTH              12
#define XDOWN_YEAST               13
#define XDOWN_YSOUTH              14
#define XDOWN_YWEST               15
#define XNORTH_YUP                16
#define XEAST_YUP                 17
#define XSOUTH_YUP                18
#define XWEST_YUP                 19
#define XNORTH_YDOWN              20
#define XEAST_YDOWN               21
#define XSOUTH_YDOWN              22
#define XWEST_YDOWN               23

#define AXIS_ROTATION_COUNT       24

class IMU
{
    public:
        //  IMUs should always be created with the following call
        static IMU *createIMU(Settings *settings);

        //  Constructor/destructor
        IMU(Settings *settings);
        virtual ~IMU();

        //  These functions must be provided by sub classes
        virtual bool IMUInit() = 0;
        virtual int IMUGetPollInterval() = 0;                   // returns the recommended poll interval in mS
        virtual bool IMURead() = 0;  

        //  the following function can be called to set the SLERP power
        void setSlerpPower(float power) { m_fusion->setSlerpPower(power); }        
        
        //  call the following to reset the fusion algorithm
        void resetFusion() { m_fusion->reset(); }

        //  the following three functions control the influence of the gyro, accel and compass sensors
        void setGyroEnable(bool enable) { m_fusion->setGyroEnable(enable);}
        void setAccelEnable(bool enable) { m_fusion->setAccelEnable(enable);}
        
        //  getIMUData returns the standard outputs of the IMU and fusion filter
        const IMU_DATA& getIMUData() { return m_imuData; }

        //
        void convertToTemperature(unsigned char *rawData);
        inline void setTemperature(const float val) { m_imuData.temperature = val; }

        //  the following two functions get access to the measured pose (accel and compass)
        const Vector3& getMeasuredPose() { return m_fusion->getMeasuredPose(); }
        const Quaternion& getMeasuredQPose() { return m_fusion->getMeasuredQPose(); }

        const Vector3& getGyro() { return m_imuData.gyro; }   // gets gyro rates in radians/sec
        const Vector3& getAccel() { return m_imuData.accel; } // get accel data in gs

        const int& get_error_cnt() { return m_error_count; } // get accel data in gs

        Vector3 getAccelResiduals() { return m_fusion->getAccelResiduals(); }

    protected:
        void gyroBiasInit();                                    // sets up gyro bias calculation
        void handleGyroBias();                                  // adjust gyro for bias
        void updateFusion();                                    // call when new data to update fusion state

        IMU_DATA m_imuData;             // the data from the IMU
        Fusion *m_fusion;               // the fusion algorithm
        Settings *m_settings;           // the settings object pointer

        uint64_t m_sampleInterval = 0;                              // interval between samples in microseonds
        int m_sampleRate = 0;      

        float m_gyroLearningAlpha;                            // gyro bias rapid learning rate
        float m_gyroContinuousAlpha;                          // gyro bias continuous (slow) learning rate
        int m_gyroSampleCount;                                  // number of gyro samples used    
        Vector3 m_previousAccel;                              // previous step accel for gyro learning

        static float m_axisRotation[AXIS_ROTATION_COUNT][9];    // array of rotation matrices

        int m_error_count = 0;

};

#endif // _IMU_H
