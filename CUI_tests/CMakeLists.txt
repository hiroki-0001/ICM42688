add_executable(main main.cpp)
target_link_libraries(main IMULIB)

add_executable(calibration calibration.cpp)
target_link_libraries(calibration IMULIB)