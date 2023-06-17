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
#include <chrono>
#include <thread>
#include <vector>
#include "ICM42688.hpp"
using namespace std::literals::chrono_literals;
int main(void)
{
  auto pid = getpid();
  auto err = setpriority(PRIO_PROCESS, pid, -20);
  if (err == -1)
  {
    std::cout << strerror(errno) << std::endl;
  }

  Settings *m_settings = new Settings("ICM42688");
  IMU *imu = IMU::createIMU(m_settings);
  imu->IMUInit();
  imu->setSlerpPower(0.2);

  std::vector<int64_t> time_list;
  std::vector<int64_t> loop_time_list;
  time_list.reserve(5000);
  loop_time_list.reserve(5000);


  // read the fifo buffer from the IMU
  volatile size_t cnt = 0;
  // while( cnt < 500)
  while(true)
  {
     auto start = std::chrono::high_resolution_clock::now();
    volatile bool ok =  imu->IMURead();
     auto end = std::chrono::high_resolution_clock::now();
    time_list.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
    cnt++;
    // std::this_thread::sleep_for(1ms);
    // end = std::chrono::high_resolution_clock::now();
    // loop_time_list.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
  }
  size_t i = 0;
  size_t total = 0;
    for(const auto itr:time_list){
        std::cout << itr << " [us] " << std::endl;
        total += itr;
        i++;
    }
    std::cerr << "average " << total / 500 << " [us]" << std::endl;
  return 0;
}