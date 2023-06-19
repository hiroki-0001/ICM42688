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

#include "ICM42688.hpp"
#include "Settings.hpp"

ICM42688::ICM42688(Settings *settings) : IMU(settings)
{
}

ICM42688::~ICM42688()
{
}

bool ICM42688::IMUInit()
{
    m_slaveAddr = m_settings->m_I2CSlaveAddress;

    if(!m_settings->HALOpen())
        return false;

    // set User Bank
    if(!setBank(0))
        return false;

    // ICM42688 reset
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_DEVICE_CONFIG, 0x01, "Failed to reset IMU"))
        return false;
    m_settings->delayMs(100);

    // ICM42688 connection check
    if(!who_i_am())
        return false;

    // enable Accel & Gyro sensor
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_PWR_MGMT0, 0x0F, "Failed to set Power mode"))
        return false;
    m_settings->delayMs(100);

    // set UI filter
    setUIFilter();

    // set Low Pass Filter
    if(!setAccelLowPassFilter(m_settings->m_ICM42688AccelLPF))
        return false;

    if(!setGyroLowPassFilter(m_settings->m_ICM42688GyroLPF))
        return false;

    // set Full Scale Range
    if(!setAccelResolutionScale(m_settings->m_ICM42688AccelFSR))
        return false;

    if(!setGyroResolutionScale(m_settings->m_ICM42688GyroFSR))
        return false;

    // set Output Data Rate
    if(!setAccelOutputDataRate(m_settings->m_ICM42688AccelODR))
        return false;

    if(!setGyroOutputDataRate(m_settings->m_ICM42688GyroODR))
        return false;

    // set sample rate [μ/sec]
    if(!setSampleRate(230.7))
        return false;

    // // enable FIFO mode
    // if(!enableFifo())
    //     return false;

    gyroBiasInit();
    
    return true;

}

bool ICM42688::setBank(uint8_t bank)
{
    if(_bank == bank)
    {
        HAL_INFO("already set bank");
        return true;
    }
    else
    {
        _bank = bank;
        if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::REG_BANK_SEL, bank, "Failed to Write set User Bank"))
            return false;
        return true;
    }
}

bool ICM42688::who_i_am()
{
    uint8_t reg = 0;
    if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_WHO_AM_I, 1, &reg, "Failed to Read ICM42688 ID"))
        return false;

    if(reg == WHO_AM_I)
    {
        HAL_INFO("ICM42688 connection successful !!!");
        return true;
    }
    else
    {
        HAL_INFO("ICM42688 connection failed !!!");
        return false;
    }
}

bool ICM42688::setSampleRate(int rate)
{
    m_sampleRate = rate;
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool ICM42688::setUIFilter()
{
    uint8_t reg, reg2;
    if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_CONFIG1, 1, &reg, "Failed Read to UB0_REG_GYRO_CONFIG1"))
        return false;

    if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_ACCEL_CONFIG1, 1, &reg2, "Failed Read to UB0_REG_GYRO_CONFIG1"))
        return false;

    return true;
}

bool ICM42688::setAccelLowPassFilter(uint8_t lpf)
{
    uint8_t reg;
    if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, 1, &reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    reg = (lpf << 4) | (reg & 0x0F);

    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    return true;
}

bool ICM42688::setGyroLowPassFilter(uint8_t lpf)
{
    uint8_t reg;
    if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, 1, &reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    reg = lpf | (reg & 0xF0);

    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    return true;
}

bool ICM42688::setAccelResolutionScale(uint8_t fssel)
{
    switch (fssel)
    {
    case ACCEL_FSR_16:
        _accelScale = 1.0/2048.0;
        break;
    case ACCEL_FSR_8:
        _accelScale = 1.0/4096.0;
        break;
    case ACCEL_FSR_4:
        _accelScale = 1.0/8192.0;
        break;
    case ACCEL_FSR_2:
        _accelScale = 1.0/16384.0;
        break;    
    default:
        HAL_ERROR("Failed to set Accel FSR");
        return false;
    }

    uint8_t reg = 0;
    if(!m_settings -> HALRead(m_slaveAddr, ICM42688reg::UB0_REG_ACCEL_CONFIG0, 1, &reg, "Failed to Read Accel Resolution Scale"))
        return false;
    // onle change FSR in reg
    reg = (fssel << 5) | (reg & 0x1F);
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_ACCEL_CONFIG0, reg, "Failed to set Accel Resolution Scale"))
        return false;

    _accelFS = fssel;

    return true;
}
bool ICM42688::setGyroResolutionScale(uint8_t fssel)
{
    switch(fssel)
    {
        case GYRO_FSR_2000:
            _gyroScale = M_PI / (16.4 * 180.0);
            break;
        case GYRO_FSR_1000:
            _gyroScale = M_PI / (32.8 * 180.0);
            break;
        case GYRO_FSR_500:
            _gyroScale = M_PI / (62.5 * 180.0);
            break;
        case GYRO_FSR_250:
            _gyroScale = M_PI / (131.0 * 180.0);
            break;
        case GYRO_FSR_125:
            _gyroScale = M_PI / (262.1 * 180.0);
            break;
        case GYRO_FSR_62_5:
            _gyroScale = M_PI / (524.3 * 180.0);
            break;
        case GYRO_FSR_31_25:
            _gyroScale = M_PI / (1048.6 * 180.0);
            break;
        case GYRO_FSR_15_625:
            _gyroScale = M_PI / (2097.2 * 180.0);
            break;
        default:
            HAL_ERROR("Failed to set Gyro FSR");
            return false;
    }

    uint8_t reg = 0;
    if(!m_settings -> HALRead(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_CONFIG0, 1, &reg, "Failed to Read Gyro Resolution Scale"))
        return false;
    // onle change FSR in reg
    reg = (fssel << 5) | (reg & 0x1F);
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_CONFIG0, reg, "Failed to set Gyro Resolution Scale"))
        return false;
    
    _gyroFS = fssel;
    
    return true;
}

bool ICM42688::setAccelOutputDataRate(uint8_t odr)
{
    uint8_t reg = 0;
    if(!m_settings -> HALRead(m_slaveAddr, ICM42688reg::UB0_REG_ACCEL_CONFIG0,  1, &reg, "Failed to Read Accel Output Data Rate"))
        return false;
    // onle change ODR in reg
    reg = odr | (reg & 0xF0);
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_ACCEL_CONFIG0, reg, "Failed to set Accel Output Data Rate"))
        return false;
    
    return true;
}
bool ICM42688::setGyroOutputDataRate(uint8_t odr)
{
    uint8_t reg = 0;
    if(!m_settings -> HALRead(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_CONFIG0,  1, &reg, "Failed to Read Gyro Output Data Rate"))
        return false;
    // onle change ODR in reg
    reg = odr | (reg & 0xF0);
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_GYRO_CONFIG0, reg, "Failed to set Gyro Output Data Rate"))
        return false;

    return true;
}

int ICM42688::IMUGetPollInterval()
{
    if (m_sampleRate > 400)
        return 1;
    else
        return (400 / m_sampleRate);
}

bool ICM42688::enableFifo()
{
    //FIFOの有効化 0x40でStream-to-FIFO
    //Stream-to-FIFO = FIFO満タン時に追加の書き込みを行う。その際には最も古いデータが置き換わる
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_FIFO_CONFIG, 0x40, "Failed to set Stream-to-FIFO"))
        return false;
    // FIFOのbufferに入れるデータの選択
    //加速度x, y, z + ジャイロx, y, z + 温度 の 7つのデータを取得する。(byte数で換算すると 14 byte) 
    if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_FIFO_CONFIG1, 0x07, "Failed to FIFO enable"))
        return false;

    return true;
}

// bool ICM42688::IMURead()
// {
//     // FIFOで使用可能なバイト数を読み込む。 High bitと Low bitの2つに分けられているので、まとめて取得する。
//     unsigned char fifoCount[2];
//     unsigned int count = 0;
//     unsigned char fifodata[16];

//     if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_FIFO_COUNTH, 2, fifoCount, "Failed to Read FIFO COUNT"))
//         return false;

//     count = (((uint16_t) fifoCount[0]) << 8) + (((uint16_t) fifoCount[1]));

//     // FIFOのオーバーフローの処理　2048byteに到達した際、FIFOのbufferを空にする。
//     if(count == 2048)
//     {
//         if(!m_settings->HALWrite(m_slaveAddr, ICM42688reg::UB0_REG_SIGNAL_PATH_RESET, 0x02, "Failed to FIFO buffer flush"))
//             return false;
//         return false;
//     }

//     while(count > _fifoFrameSize)
//     {
//         if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_FIFO_DATA, _fifoFrameSize, fifodata, "Failed to Read FIFO data"))
//             return false;
//         count -= _fifoFrameSize;
//     }

//     // FIFOの利用可能なbyte数がFIFOのFrameSize(16 byte)よりも少ない場合は、データのbyteがずれるおそれがあるため
//     // 使用しないようにする
//     if(count < _fifoFrameSize)
//     {
//         m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_FIFO_DATA, count, fifodata, "Failed to Read FIFO data");
//         return false;
//     }

//     // 上記の条件が一致しない場合は正常であるため、bufferからデータを読み込み、使用する
//     if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_FIFO_DATA, _fifoFrameSize, fifodata, "Failed to Read fifo data"))
//         return false;

//     Vector3::convertToVector(fifodata + 1, m_imuData.accel, _accelScale);
//     Vector3::convertToVector(fifodata + 7, m_imuData.gyro, _gyroScale);
//     IMU::convertToTemperature(fifodata + 13);

//     handleGyroBias();


//     m_imuData.timestamp = IMUMath::currentUSecsSinceEpoch();

//     updateFusion();

//     return true;
// }


bool ICM42688::IMURead()
{
    uint8_t count = 12;
    if(!m_settings->HALRead(m_slaveAddr, ICM42688reg::UB0_REG_ACCEL_DATA_X1, count, _buffer, "error"))
        return false;

    Vector3::convertToVector(_buffer, m_imuData.accel, _accelScale);
    Vector3::convertToVector(_buffer + 6, m_imuData.gyro, _gyroScale);

    handleGyroBias();
    m_imuData.timestamp = IMUMath::currentUSecsSinceEpoch();
    updateFusion();

    return true;
}