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

class IMU
{
    public:
        IMU();
        virtual ~IMU();

        void setSlerpPower(float power) { m_fusion->setSlerpPower(power); }        
        //  call the following to reset the fusion algorithm
        void resetFusion() { m_fusion->reset(); }
        // void resetFusion() {}

        void setGyroEnable(bool enable) { m_fusion->setGyroEnable(enable);}
        void setAccelEnable(bool enable) { m_fusion->setAccelEnable(enable);}
        //
        virtual int IMUGetPollInterval() = 0;                   // returns the recommended poll interval in mS
        virtual bool IMURead() = 0;  
        //  getIMUData returns the standard outputs of the IMU and fusion filter
        const IMU_DATA& getIMUData() { return m_imuData; }
        //
        void convertToTemperature(unsigned char *rawData);
        inline void setTemperature(const float val) { m_imuData.temperature = val; }

        Vector3 getAccelResiduals() { return m_fusion->getAccelResiduals(); }
        const Vector3& getMeasuredPose() { return m_fusion->getMeasuredPose(); }

        uint64_t getSampleRate() {return m_sampleRate;}


    protected:
        static char m_string[1000];
        IMU_DATA m_imuData;                                   // the data from the IMU
        Fusion *m_fusion;     
        void updateFusion();                                    // call when new data to update fusion state
        uint64_t m_sampleInterval = 0;                              // interval between samples in microseonds
        int m_sampleRate = 0;      
};

#endif // _IMU_H
