add_executable(main main.cpp)
target_link_libraries(main ICM42688_Lib)

add_executable(clock clock.cpp)
target_link_libraries(clock ICM42688_Lib)
target_compile_options(clock PRIVATE -O2 -march=native)