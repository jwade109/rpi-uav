cmake_minimum_required(VERSION 3.1)

project(rpi-uav)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include/")

add_library(control
    ${CMAKE_SOURCE_DIR}/src/control/controller.cpp)

add_library(algorithm
    ${CMAKE_SOURCE_DIR}/src/algorithm/pid.cpp
    ${CMAKE_SOURCE_DIR}/src/algorithm/gps_baro_filter.cpp
    ${CMAKE_SOURCE_DIR}/src/algorithm/gps_position_filter.cpp
    ${CMAKE_SOURCE_DIR}/src/algorithm/mass_estimator.cpp
    ${CMAKE_SOURCE_DIR}/src/algorithm/motor_director.cpp)

add_library(math
    ${CMAKE_SOURCE_DIR}/src/math/freebody.cpp
    ${CMAKE_SOURCE_DIR}/src/math/angle.cpp
    ${CMAKE_SOURCE_DIR}/src/math/coordinate.cpp
    ${CMAKE_SOURCE_DIR}/src/math/functions.cpp)

add_library(filter
    ${CMAKE_SOURCE_DIR}/src/filter/filters.cpp)

add_library(hardware
    ${CMAKE_SOURCE_DIR}/src/hardware/i2c.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/pwm.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/ardimu.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/bmp.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/gps.cpp
    ${CMAKE_SOURCE_DIR}/src/hardware/sensors.cpp)
target_link_libraries(hardware -pthread)
target_link_libraries(hardware -latomic)
target_link_libraries(hardware -lwiringPi)

add_library(logging
    ${CMAKE_SOURCE_DIR}/src/logging/archive.cpp
    ${CMAKE_SOURCE_DIR}/src/logging/logstream.cpp)

add_executable(gpstest
    ${CMAKE_SOURCE_DIR}/test/gpstest.cpp)
target_link_libraries(gpstest math)
target_link_libraries(gpstest hardware)
target_link_libraries(gpstest filter)

add_executable(sensortest
    ${CMAKE_SOURCE_DIR}/test/units.cpp)
target_link_libraries(sensortest hardware)
target_link_libraries(sensortest math)

add_executable(serialtest
    ${CMAKE_SOURCE_DIR}/test/serialtest.cpp)
target_link_libraries(serialtest hardware)
target_link_libraries(serialtest math)

add_executable(unit
    ${CMAKE_SOURCE_DIR}/src/tests/test_main.cpp
    ${CMAKE_SOURCE_DIR}/src/tests/math.cpp
    ${CMAKE_SOURCE_DIR}/src/tests/logging.cpp)
target_link_libraries(unit math)
target_link_libraries(unit logging)

add_executable(launch
    ${CMAKE_SOURCE_DIR}/src/control/launch.cpp)
target_link_libraries(launch control)
target_link_libraries(launch math)
target_link_libraries(launch logging)
target_link_libraries(launch hardware)
target_link_libraries(launch algorithm)
target_link_libraries(launch filter)
