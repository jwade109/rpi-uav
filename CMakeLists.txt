cmake_minimum_required(VERSION 3.1)

project(rpi-uav)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include/")

add_library(lcontrol
    ${CMAKE_SOURCE_DIR}/src/control/control.cpp)

add_library(lalgorithm
    ${CMAKE_SOURCE_DIR}/src/algorithm/pid.cpp)

add_library(lmath
    ${CMAKE_SOURCE_DIR}/src/math/freebody.cpp
    ${CMAKE_SOURCE_DIR}/src/math/angle.cpp
    ${CMAKE_SOURCE_DIR}/src/math/functions.cpp)

add_library(lfilter
    ${CMAKE_SOURCE_DIR}/src/filter/filters.cpp)

add_library(lhardware
    ${CMAKE_SOURCE_DIR}/src/hardware/i2c.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/pwm.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/ardimu.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/bmp.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/gps.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/sensors.cpp)
target_link_libraries(lhardware -pthread)
target_link_libraries(lhardware -latomic)
target_link_libraries(lhardware -lwiringPi)

add_library(llogging
    ${CMAKE_SOURCE_DIR}/src/logging/logstream.cpp
    ${CMAKE_SOURCE_DIR}/src/logging/uavcore.cpp
    ${CMAKE_SOURCE_DIR}/src/math/freebody.cpp)

add_executable(launch
    ${CMAKE_SOURCE_DIR}/src/control/launch.cpp)
target_link_libraries(launch lcontrol)
target_link_libraries(launch lalgorithm)
target_link_libraries(launch lmath)
target_link_libraries(launch lhardware)
target_link_libraries(launch llogging)
target_link_libraries(launch lfilter)

add_executable(gpstest
    ${CMAKE_SOURCE_DIR}/test/gpstest.cpp)
target_link_libraries(gpstest lmath)
target_link_libraries(gpstest lhardware)
target_link_libraries(gpstest lfilter)

add_executable(bin2txt
    ${CMAKE_SOURCE_DIR}/test/bin2txt.cpp)
target_link_libraries(bin2txt lmath)
target_link_libraries(bin2txt llogging)

add_executable(bin2vel
    ${CMAKE_SOURCE_DIR}/test/bin2vel.cpp)
target_link_libraries(bin2vel lmath)
target_link_libraries(bin2vel llogging)

add_executable(skips
    ${CMAKE_SOURCE_DIR}/test/skips.cpp)
target_link_libraries(skips lmath)
target_link_libraries(skips llogging)

add_executable(mathunit
    ${CMAKE_SOURCE_DIR}/src/tests/math.cpp)
target_link_libraries(mathunit lmath)
