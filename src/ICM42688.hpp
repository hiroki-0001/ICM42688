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
  enum GyroFS : uint8_t
  {
    dps2000 = 0x00,
    dps1000 = 0x01,
    dps500 = 0x02,
    dps250 = 0x03,
    dps125 = 0x04,
    dps62_5 = 0x05,
    dps31_25 = 0x06,
    dps15_625 = 0x07
  };
  enum AccelFS : uint8_t
  {
    gpm16 = 0x00,
    gpm8 = 0x01,
    gpm4 = 0x02,
    gpm2 = 0x03
  };
  enum ODR : uint8_t
  {
    odr32k = 0x01, // LN mode only
    odr16k = 0x02, // LN mode only
    odr8k = 0x03,  // LN mode only
    odr4k = 0x04,  // LN mode only
    odr2k = 0x05,  // LN mode only
    odr1k = 0x06,  // LN mode only (Accel & Gyro default)
    odr200 = 0x07,
    odr100 = 0x08,
    odr50 = 0x09,
    odr25 = 0x0A,
    odr12_5 = 0x0B,
    odr6a25 = 0x0C,   // LP mode only (accel only)
    odr3a125 = 0x0D,  // LP mode only (accel only)
    odr1a5625 = 0x0E, // LP mode only (accel only)
    odr500 = 0x0F,
  };
  enum LPF : uint8_t
  {
    lpf_1 = 0x00,  // ODR/2, (LN mode only)
    lpf_2 = 0x01,  // LN mode : max(400Hz, ODR)/4, LP mode : 1x AVG filter (default)
    lpf_3 = 0x02,  // max(400Hz, ODR)/5, (LN mode only)
    lpf_4 = 0x03,  // max(400Hz, ODR)/8, (LN mode only)
    lpf_5 = 0x04,  // max(400Hz, ODR)/10, (LN mode only)
    lpf_6 = 0x05,  // max(400Hz, ODR)/16, (LN mode only)
    lpf_7 = 0x06,  // max(400Hz, ODR)/20, (LN mode only)
    lpf_8 = 0x07,  // max(400Hz, ODR)/40, (LN mode only)
    lpf_9 = 0x0E,  // Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)
    lpf_10 = 0x0F, // Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)
  };

  ICM42688();
  ~ICM42688();

  bool setSampleRate(int rate);

  bool begin();
  int setBank(uint8_t bank);
  bool who_i_am();
  bool setGyroLowPassFilter(LPF lpf);
  bool setAccelLowPassFilter(LPF lpf);
  bool setAccelResolutionScale(AccelFS fssel);
  bool setGyroResolutionScale(GyroFS fssel);
  bool setAccelOutputDataRate(ODR odr);
  bool setGyroOutputDataRate(ODR odr);

  bool enableFifo(bool accel, bool gyro, bool temp);
  bool IMURead();

  int IMUGetPollInterval();

protected:
  SPI *spidev;
  float _accelScale = 0.0f;
  float _gyroScale = 0.0f;
  AccelFS _accelFS;
  GyroFS _gyroFS;
  uint8_t _buffer[12] = {};
  // Constants
  static constexpr uint8_t WHO_AM_I = 0x47;      ///< expected value in UB0_REG_WHO_AM_I reg
  static constexpr int NUM_CALIB_SAMPLES = 1000; ///< for gyro/accel bias calib
  static constexpr uint8_t READ_ADDR = 0x80;
  static constexpr uint8_t WRITE_ADDR = 0x00;
  uint8_t _bank = 0;
  
  static constexpr uint8_t FIFO_ACCEL = 0x01;
  static constexpr uint8_t FIFO_GYRO = 0x02;
  static constexpr uint8_t FIFO_TEMP = 0x04;

  bool _enFifoAccel = false;
  bool _enFifoGyro = false;
  bool _enFifoTemp = false;

  size_t _fifoSize = 0;
  size_t _fifoFrameSize = 0;

  private:
      bool m_firstTime_ICM42688 = true;
};
#endif // ICM42688_H