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

#include <unistd.h>
#include <sched.h>
#include <sys/resource.h>
#include "ICM42688.hpp"

int main(void)
{
  auto pid = getpid();
  auto err = setpriority(PRIO_PROCESS, pid, -20);
  if (err == -1)
  {
    std::cout << strerror(errno) << std::endl;
  }

  ICM42688 imu;
  imu.begin();

  // read the fifo buffer from the IMU
  while(1)
  {
    if(imu.IMURead())
    {
      std::cout << "timestamp = " << imu.getIMUData().timestamp << std::endl;
      std::cout << "acc x = " << imu.getIMUData().accel.x() << std::endl;
      std::cout << "acc y = " << imu.getIMUData().accel.y() << std::endl;
      std::cout << "acc z = " << imu.getIMUData().accel.z() << std::endl;
      std::cout << "gyro x = " << imu.getIMUData().gyro.x() * RAD_TO_DEGREE << std::endl;
      std::cout << "gyro y = " << imu.getIMUData().gyro.y() * RAD_TO_DEGREE << std::endl;
      std::cout << "gyro z = " << imu.getIMUData().gyro.z() * RAD_TO_DEGREE << std::endl;
      std::cout << "temperture = " << imu.getIMUData().temperature << std::endl;
      std::cout << "-------------------" << std::endl;
      std::cout << "fusion vector x = " << imu.getIMUData().fusionPose.x() * RAD_TO_DEGREE << std::endl;
      std::cout << "fusion vector y = " << imu.getIMUData().fusionPose.y() * RAD_TO_DEGREE << std::endl;
      std::cout << "fusion vector z = " << imu.getIMUData().fusionPose.z() * RAD_TO_DEGREE << std::endl;
      std::cout << "fusion quaternion scalar = " << imu.getIMUData().fusionQPose.scalar() << std::endl;
      std::cout << "fusion quaternion x = " << imu.getIMUData().fusionQPose.x() << std::endl;
      std::cout << "fusion quaternion y = " << imu.getIMUData().fusionQPose.y() << std::endl;
      std::cout << "fusion quaternion z = " << imu.getIMUData().fusionQPose.z() << std::endl;
      std::cout << std::endl;

      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  return 0;
}