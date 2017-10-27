cmake_minimum_required(VERSION 3.1)

project(rpi-uav)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include/")

add_executable(bin2txt
    ${CMAKE_SOURCE_DIR}/test/bin2txt.cpp
    ${CMAKE_SOURCE_DIR}/src/logging/uavcore.cpp
    ${CMAKE_SOURCE_DIR}/src/math/freebody.cpp)

add_executable(bin2vel
    ${CMAKE_SOURCE_DIR}/test/bin2vel.cpp
    ${CMAKE_SOURCE_DIR}/src/logging/uavcore.cpp
    ${CMAKE_SOURCE_DIR}/src/math/freebody.cpp)

add_executable(skips
    ${CMAKE_SOURCE_DIR}/test/skips.cpp
    ${CMAKE_SOURCE_DIR}/src/logging/uavcore.cpp
    ${CMAKE_SOURCE_DIR}/src/math/freebody.cpp)
