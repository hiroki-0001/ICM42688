#include "ICM42688.hpp"

int main(void)
{
    ICM42688 imu;
    imu.begin();
    imu.offsetBias();
    return 0;
}