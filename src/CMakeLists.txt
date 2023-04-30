set(IMU_SRCS
        ICM42688.cpp 
        IMUMath.cpp 
        IMU.cpp 
        Fusion.cpp
        OrientationEstimator.cpp
        spidev_lib.cpp
)

add_library(IMULIB SHARED ${IMU_SRCS})

