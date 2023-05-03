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

ICM42688::ICM42688()
{
    spidev = new SPI("/dev/spidev0.0");
}

ICM42688::~ICM42688()
{
    delete spidev;
}

bool ICM42688::begin()
{
    if(spidev->begin())
    {
        if(!setBank(0))
            return false;
        // ICM42688 reset
        if(!spidev->write(ICM42688reg::UB0_REG_DEVICE_CONFIG, 0x01, "Failed to reset IMU"))
            return false;
        spidev->delayMs(10);
        // ICM42688 connection check
        if(!who_i_am())
            return false;
        // enable Accel & Gyro sensor
        if(!spidev->write(ICM42688reg::UB0_REG_PWR_MGMT0, 0x0F, "Failed to set Power mode"))
            return false;
        
        m_firstTime_ICM42688 = true;
        // set UI filter
        setUIFilter();
        // set Low Pass Filter
        setAccelLowPassFilter(lpf_2);
        setGyroLowPassFilter(lpf_2);
        // set Full Scale Range
        setAccelResolutionScale(gpm8);
        setGyroResolutionScale(dps2000);
        // set Output Data Rate
        setAccelOutputDataRate(odr200);
        setGyroOutputDataRate(odr200);
        // set sample rate [μ/sec]
        setSampleRate(92.3);

        enableFifo(true, true, true);
        setSlerpPower(0.02);
        setoffsetBias();
        
        return true;
    }
    else
    {
        return false;
    }

}

bool ICM42688::setBank(uint8_t bank)
{
    if(_bank == bank)
    {
        MESSAGE_LOG("already set bank");
        return true;
    }
    else
    {
        _bank = bank;
        if(!spidev->write(ICM42688reg::REG_BANK_SEL, bank, "Failed to write set User Bank"))
            return false;
        return true;
    }
}

bool ICM42688::who_i_am()
{
    uint8_t reg = 0;
    if(!spidev->read(ICM42688reg::UB0_REG_WHO_AM_I, 1, &reg, "Failed to read ICM42688 ID"))
        return false;

    if(reg == WHO_AM_I)
    {
        MESSAGE_LOG("ICM42688 connection successful !!!");
        return true;
    }
    else
    {
        MESSAGE_LOG("ICM42688 connection failed !!!");
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
    if(!spidev->read(ICM42688reg::UB0_REG_GYRO_CONFIG1, 1, &reg, "Failed read to UB0_REG_GYRO_CONFIG1"))
        return false;

    if(!spidev->read(ICM42688reg::UB0_REG_ACCEL_CONFIG1, 1, &reg2, "Failed read to UB0_REG_GYRO_CONFIG1"))
        return false;

    std::cout << "gyro_config1 = " << std::bitset<8>(reg) << std::endl;
    std::cout << "accel_config1 = " << std::bitset<8>(reg2) << std::endl;
    return true;
}

bool ICM42688::setAccelLowPassFilter(LPF lpf)
{
    uint8_t reg;
    if(!spidev->read(ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, 1, &reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    reg = (lpf << 4) | (reg & 0x0F);

    if(!spidev->write(ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    return true;
}

bool ICM42688::setGyroLowPassFilter(LPF lpf)
{
    uint8_t reg;
    if(!spidev->read(ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, 1, &reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    reg = lpf | (reg & 0xF0);

    if(!spidev->write(ICM42688reg::UB0_REG_GYRO_ACCEL_CONFIG0, reg, "Failed to set Accel Low-Pass Filter"))
        return false;

    return true;
}

bool ICM42688::setAccelResolutionScale(AccelFS fssel)
{
    switch (fssel)
    {
    case gpm16:
        _accelScale = 1.0/2048.0;
        break;
    case gpm8:
        _accelScale = 1.0/4096.0;
        break;
    case gpm4:
        _accelScale = 1.0/8192.0;
        break;
    case gpm2:
        _accelScale = 1.0/16384.0;
        break;    
    default:
        ERROR_LOG("Failed to set Accel FSR");
        return false;
    }

    uint8_t reg = 0;
    if(!spidev -> read(ICM42688reg::UB0_REG_ACCEL_CONFIG0, 1, &reg, "Failed to read Accel Resolution Scale"))
        return false;
    // onle change FSR in reg
    reg = (fssel << 5) | (reg & 0x1F);
    if(!spidev->write(ICM42688reg::UB0_REG_ACCEL_CONFIG0, reg, "Failed to set Accel Resolution Scale"))
        return false;

    _accelFS = fssel;

    return true;
}
bool ICM42688::setGyroResolutionScale(GyroFS fssel)
{
    switch(fssel)
    {
        case dps2000:
            _gyroScale = M_PI / (16.4 * 180.0);
            break;
        case dps1000:
            _gyroScale = M_PI / (32.8 * 180.0);
            break;
        case dps500:
            _gyroScale = M_PI / (62.5 * 180.0);
            break;
        case dps250:
            _gyroScale = M_PI / (131.0 * 180.0);
            break;
        case dps125:
            _gyroScale = M_PI / (262.1 * 180.0);
            break;
        case dps62_5:
            _gyroScale = M_PI / (524.3 * 180.0);
            break;
        case dps31_25:
            _gyroScale = M_PI / (1048.6 * 180.0);
            break;
        case dps15_625:
            _gyroScale = M_PI / (2097.2 * 180.0);
            break;
        default:
            ERROR_LOG("Failed to set Gyro FSR");
            return false;
    }

    uint8_t reg = 0;
    if(!spidev -> read(ICM42688reg::UB0_REG_GYRO_CONFIG0, 1, &reg, "Failed to read Gyro Resolution Scale"))
        return false;
    // onle change FSR in reg
    reg = (fssel << 5) | (reg & 0x1F);
    if(!spidev->write(ICM42688reg::UB0_REG_GYRO_CONFIG0, reg, "Failed to set Gyro Resolution Scale"))
        return false;
    
    _gyroFS = fssel;
    
    return true;
}

bool ICM42688::setAccelOutputDataRate(ODR odr)
{
    uint8_t reg = 0;
    if(!spidev -> read(ICM42688reg::UB0_REG_ACCEL_CONFIG0,  1, &reg, "Failed to read Accel Output Data Rate"))
        return false;
    // onle change ODR in reg
    reg = odr | (reg & 0xF0);
    if(!spidev->write(ICM42688reg::UB0_REG_ACCEL_CONFIG0, reg, "Failed to set Accel Output Data Rate"))
        return false;

    return true;
}
bool ICM42688::setGyroOutputDataRate(ODR odr)
{
    uint8_t reg = 0;
    if(!spidev -> read(ICM42688reg::UB0_REG_GYRO_CONFIG0,  1, &reg, "Failed to read Gyro Output Data Rate"))
        return false;
    // onle change ODR in reg
    reg = odr | (reg & 0xF0);
    if(!spidev->write(ICM42688reg::UB0_REG_GYRO_CONFIG0, reg, "Failed to set Gyro Output Data Rate"))
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

bool ICM42688::enableFifo(bool accel,bool gyro, bool temp)
{
    //FIFOの有効化 0x40でStream-to-FIFO
    //Stream-to-FIFO = FIFO満タン時に追加の書き込みを行う。その際には最も古いデータが置き換わる
    if(!spidev->write(ICM42688reg::UB0_REG_FIFO_CONFIG, 0x40, "Failed to set Stream-to-FIFO"))
        return false;
    // FIFOのbufferに入れるデータの選択
    //加速度x, y, z + ジャイロx, y, z + 温度 の 7つのデータを取得する。(byte数で換算すると 14 byte) 
    if(!spidev->write(ICM42688reg::UB0_REG_FIFO_CONFIG1, (accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(temp*FIFO_TEMP), "Failed to FIFO enable"))
        return false;
    
    _enFifoAccel = accel;
    _enFifoGyro = gyro;
    _enFifoTemp = temp;

    // FIFO packet stracture
    // FIFO の 中身の構造 前述したデータ14byteに加えて、FIFOの情報を示す Header (1 byte), timestamp(2 byte)が含まれる。
    // Header(1) + accel(6) + gyro(6) + temp(1) + timestamp(2) 
    _fifoFrameSize = 1 + accel*6 + gyro*6 + temp*1 + 2;
    return true;
}

bool ICM42688::IMURead()
{
    // FIFOで使用可能なバイト数を読み込む。 High bitと Low bitの2つに分けられているので、まとめて取得する。
    uint8_t reg[2] = {};
    if(!spidev->read(ICM42688reg::UB0_REG_FIFO_COUNTH, 2, reg, "Failed to read FIFO COUNT"))
        return false;

    _fifoSize = (((uint16_t) reg[0]) << 8) + (((uint16_t) reg[1]));

    // FIFOのオーバーフローの処理　2048byteに到達した際、FIFOのbufferを空にする。
    if(_fifoSize >= 2048)
    {
        if(!spidev->write(ICM42688reg::UB0_REG_SIGNAL_PATH_RESET, 0x02, "Failed to FIFO buffer flush"))
            return false;
        MESSAGE_LOG("fifo full to flush")
        m_imuData.timestamp += m_sampleInterval * (2048  / _fifoFrameSize + 1); // try to fix timestamp
        return false;
    }

    while(_fifoSize > _fifoFrameSize)
    {
        if(!spidev->read(ICM42688reg::UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer, "Failed to read FIFO data"))
            return false;
        _fifoSize -= _fifoFrameSize;
        m_imuData.timestamp += m_sampleInterval;
    }

    if(_fifoSize < _fifoFrameSize)
    {
        spidev->read(ICM42688reg::UB0_REG_FIFO_DATA, _fifoSize, _buffer, "Failed to read FIFO data");
        MESSAGE_LOG("fifo storage less");
        return false;
    }

    if(_enFifoAccel)
        Vector3::convertToVector(_buffer + 1, m_imuData.accel, _accelScale, _accelBias);

    if(_enFifoGyro)
        Vector3::convertToVector(_buffer + 7, m_imuData.gyro, _gyroScale, _gyroBias);

    if(_enFifoTemp)
        IMU::convertToTemperature(_buffer + 13);

    if (m_firstTime_ICM42688)
        m_imuData.timestamp = IMUMath::currentUSecsSinceEpoch();
    else
        m_imuData.timestamp += m_sampleInterval;

    m_firstTime_ICM42688 = false;

    updateFusion();
    
    return true;
}

// bool ICM42688::IMURead()
// {
//     uint8_t count = 12;
//     spidev->read(ICM42688reg::UB0_REG_ACCEL_DATA_X1, count, _buffer, "error");
//     Vector3::convertToVector(_buffer, m_imuData.accel,_accelScale, _accelBias);
//     Vector3::convertToVector(_buffer + 6, m_imuData.gyro,_gyroScale, _gyroBias);

//     if (m_firstTime_ICM42688)
//         m_imuData.timestamp = IMUMath::currentUSecsSinceEpoch();
//     else
//         m_imuData.timestamp += m_sampleInterval;

//     m_firstTime_ICM42688 = false;

//     updateFusion();

//     return true;
// }

// FIFOなしで加速度・ジャイロの値を取得するときに使用。
// calibration用に使用する

bool ICM42688::readData(int16_t *data)
{
    if(!spidev->read(ICM42688reg::UB0_REG_ACCEL_DATA_X1, 12, _buffer, "Failed to read data of accel & gyro"))
        return false;

    data[0] = (uint16_t)_buffer[0] << 8 | _buffer[1];
    data[1] = (uint16_t)_buffer[2] << 8 | _buffer[3];
    data[2] = (uint16_t)_buffer[4] << 8 | _buffer[5];
    data[3] = (uint16_t)_buffer[6] << 8 | _buffer[7];
    data[4] = (uint16_t)_buffer[8] << 8 | _buffer[9];
    data[5] = (uint16_t)_buffer[10] << 8 | _buffer[11];

    return true;
}

bool ICM42688::offsetBias()
{   
    std::cout << "calibration start !!" << std::endl;
    //キャリブレーション中はIMUを動かさないので、分解能を高く設定する
    const AccelFS accel_current_fssel = _accelFS;
    const GyroFS gyro_current_fssel = _gyroFS;
    
    if(!setAccelResolutionScale(gpm2))
        return false;
    
    if(!setGyroResolutionScale(dps250))
        return false;

    static bool called_first_time = true;

    int16_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t sum[6] = {0, 0, 0, 0, 0, 0};
    
    // IMUのデータ取得の際、最初に取得するデータはノイズが大きいため排除する
    if(called_first_time)
    {
        readData(data);
        spidev->delayMs(50);
        called_first_time = false;
    }

    for(int i = 0; i < 128; i++)
    {
        readData(data);
        sum[0] += data[0];
        sum[1] += data[1];
        sum[2] += data[2];
        sum[3] += data[3];
        sum[4] += data[4];
        sum[5] += data[5];
        spidev->delayMs(50);
    }

    _accelBias[0] = sum[0] * _accelScale / 128.0f;
    _accelBias[1] = sum[1] * _accelScale / 128.0f;
    _accelBias[2] = sum[2] * _accelScale / 128.0f;
    _gyroBias[0] = sum[3] * _gyroScale / 128.0f;
    _gyroBias[1] = sum[4] * _gyroScale / 128.0f;
    _gyroBias[2] = sum[5] * _gyroScale / 128.0f;

    if(_accelBias[0] > 0.8f)  {_accelBias[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(_accelBias[0] < -0.8f) {_accelBias[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(_accelBias[1] > 0.8f)  {_accelBias[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(_accelBias[1] < -0.8f) {_accelBias[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(_accelBias[2] > 0.8f)  {_accelBias[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
    if(_accelBias[2] < -0.8f) {_accelBias[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation

    std::cout << " acc x bias = " <<  _accelBias[0] << std::endl;
    std::cout << " acc y bias = " <<  _accelBias[1] << std::endl;
    std::cout << " acc z bias = " <<  _accelBias[2] << std::endl;
    std::cout << " gyro x bias = " <<  _gyroBias[0] << std::endl;
    std::cout << " gyro y bias = " <<  _gyroBias[1] << std::endl;
    std::cout << " gyro z bias = " <<  _gyroBias[2] << std::endl;

    // recover the full scale setting
    if(!setAccelResolutionScale(accel_current_fssel))
        return false;

    if(!setGyroResolutionScale(gyro_current_fssel))
        return false;
    
    return true;
}

void ICM42688::setoffsetBias()
{
  // offset bias
  float accBias[3] =
  {
    -8.86917e-05,
    -0.021184,
    0.0101852
  };

  float gyroBias[3] = 
  {
    -0.00566753,
    -0.00247519,
    -0.00345152
  };

    for(int i = 0; i < 3; i++)
    {
        _accelBias[i] = accBias[i];
        _gyroBias[i] = gyroBias[i];
    }
}
