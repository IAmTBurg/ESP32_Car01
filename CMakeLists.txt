# ./CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# include IDF_PATH
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Specify Project Version
project(ESP32_Car03 VERSION 1.0.0)

# Add global include directory
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/components/motor_control/include
    ${CMAKE_SOURCE_DIR}/components/my_i2c_interface/include
    ${CMAKE_SOURCE_DIR}/components/pca9685/include
    ${CMAKE_SOURCE_DIR}/components/sbus_component/include
)

# Add components
add_subdirectory(components/motor_control)
add_subdirectory(components/my_i2c_interface)
add_subdirectory(components/pca9685)
add_subdirectory(components/sbus_component)
add_subdirectory(src)