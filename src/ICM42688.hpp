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

#ifndef ICM42688_H
#define ICM42688_H

#include <iostream>
#include <bitset>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <cstring>
#include "IMU.hpp"
#include "spidev_lib.hpp"

class ICM42688 : public IMU
{
public:
  ICM42688(Settings *settings);
  ~ICM42688();

  bool setSampleRate(int rate);

  bool IMUInit();
  bool setBank(uint8_t bank);
  bool who_i_am();
  bool setUIFilter();
  bool setGyroLowPassFilter(uint8_t lpf);
  bool setAccelLowPassFilter(uint8_t lpf);
  bool setAccelResolutionScale(uint8_t fssel);
  bool setGyroResolutionScale(uint8_t fssel);
  bool setAccelOutputDataRate(uint8_t odr);
  bool setGyroOutputDataRate(uint8_t odr);

  bool enableFifo();
  bool IMURead();
  bool readData(int16_t *data);

  bool offsetBias();
  bool setoffsetBias();

  int IMUGetPollInterval();

  private:
      SPI *spidev;
      float _accelScale = 0.0f;
      float _gyroScale = 0.0f;
      uint8_t _accelFS;
      uint8_t _gyroFS;
      float _accelBias[3] = {0.0f, 0.0f, 0.0f};
      float _gyroBias[3] = {0.0f, 0.0f, 0.0f};

      // read data buffer For calibration
      uint8_t _buffer[12] = {};
      
      // Constants
      static constexpr uint8_t WHO_AM_I = 0x47;      ///< expected value in UB0_REG_WHO_AM_I reg
      static constexpr int NUM_CALIB_SAMPLES = 1000; ///< for gyro/accel bias calib
      static constexpr uint8_t READ_ADDR = 0x80;
      static constexpr uint8_t WRITE_ADDR = 0x00;
      uint8_t _bank = 0;
      
    // FIFO packet stracture
    // FIFO の 中身の構造 前述したデータ14byteに加えて、FIFOの情報を示す Header (1 byte), timestamp(2 byte)が含まれる。
    // Header(1) + accel(6) + gyro(6) + temp(1) + timestamp(2) 
      size_t _fifoFrameSize = 16;
};
#endif // ICM42688_H