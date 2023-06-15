########################################################################
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "Prevented in-tree built. This is bad practice.")
ENDIF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.18)
project(ICM42688 CXX)

option(BUILD_CUI_DEMO "Build CUI demo program." OFF)
if(BUILD_CUI_DEMO)
    add_subdirectory(CUI_demo)
endif(BUILD_CUI_DEMO)

option(BUILD_GUI_DEMO "Build GUI demo program." OFF)
if(BUILD_GUI_DEMO)
    add_subdirectory(GUI_demo)
endif(BUILD_GUI_DEMO)


add_library(ICM42688_Lib STATIC)
target_sources(ICM42688_Lib 
        PRIVATE 
            src/ICM42688.cpp
            src/IMUMath.cpp
            src/IMU.cpp
            src/Fusion.cpp
            src/spidev_lib.cpp)
set_property(TARGET ICM42688_Lib 
            PROPERTY PUBLIC_HEADER
            src/IMUMath.hpp
            src/Fusion.hpp
            src/ICM42688Defs.hpp
            src/ICM42688.hpp
            src/IMU.hpp
)
target_include_directories(ICM42688_Lib PUBLIC src)
target_compile_features(ICM42688_Lib PRIVATE cxx_std_17)
target_compile_options(ICM42688_Lib PRIVATE -Wall -O2 -march=native)