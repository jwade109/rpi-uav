cmake_minimum_required(VERSION 3.1)

project(rpi-uav)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include/")
include_directories("/usr/local/include/eigen3/")

add_subdirectory(hardware)
add_subdirectory(logging)
add_subdirectory(math)
add_subdirectory(algorithm)
add_subdirectory(control)
add_subdirectory(spike)
add_subdirectory(test)
