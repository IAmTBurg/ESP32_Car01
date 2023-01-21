cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(EXTRA_COMPONENT_DIRS components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
 
project(ESP32_Car01)


# set(EXTRA_COMPONENT_DIRS components components/gpio_component src include)