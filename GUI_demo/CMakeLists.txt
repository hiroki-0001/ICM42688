########################################################################

include(CMakeDependentOption)
include(FeatureSummary)

########################################################################
# Project setup
########################################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/RTIMULibGL)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/RTIMULibGL/QtGLLib)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/RTIMULibGL/VRWidgetLib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/RTIMULibGL)

add_subdirectory(RTIMULibDemoGL)