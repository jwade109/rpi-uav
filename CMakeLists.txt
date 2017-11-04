cmake_minimum_required(VERSION 3.1)

project(rpi-uav)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include/")

add_subdirectory(hardware)
add_subdirectory(logging)
add_subdirectory(math)
add_subdirectory(algorithm)
add_subdirectory(control)
add_subdirectory(spike)

add_library(uav
    $<TARGET_OBJECTS:hardware>
    $<TARGET_OBJECTS:logging>
    $<TARGET_OBJECTS:math>
    $<TARGET_OBJECTS:algorithm>
    $<TARGET_OBJECTS:control>)
target_link_libraries(uav -lwiringPi -lpthread -latomic)

add_executable(gpstest
    ${CMAKE_SOURCE_DIR}/test/gpstest.cpp)
target_link_libraries(gpstest uav)

add_executable(sensortest
    ${CMAKE_SOURCE_DIR}/test/units.cpp)
target_link_libraries(sensortest uav)

add_executable(serialtest
    ${CMAKE_SOURCE_DIR}/test/serialtest.cpp)
target_link_libraries(serialtest uav)

add_executable(unit
    ${CMAKE_SOURCE_DIR}/src/tests/test_main.cpp
    ${CMAKE_SOURCE_DIR}/src/tests/math.cpp
    ${CMAKE_SOURCE_DIR}/src/tests/logging.cpp)
target_link_libraries(unit uav)

add_executable(skips
    ${CMAKE_SOURCE_DIR}/test/skips.cpp)
target_link_libraries(skips uav)

add_executable(bin2txt
    ${CMAKE_SOURCE_DIR}/test/bin2txt.cpp)
target_link_libraries(bin2txt uav)

add_executable(print_events
    ${CMAKE_SOURCE_DIR}/test/events.cpp)
target_link_libraries(print_events uav)
