include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(hardware
    ${CMAKE_CURRENT_LIST_DIR}/src/ardimu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/bmp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gps.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/i2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pwm.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sensors.cpp)
target_link_libraries(hardware -lwiringPi -lpthread -latomic)

add_executable(gps_test
    ${CMAKE_CURRENT_LIST_DIR}/test/gps_test.cpp)
target_link_libraries(gps_test hardware math)
add_executable(bmp_test
    ${CMAKE_CURRENT_LIST_DIR}/test/bmp_test.cpp)
target_link_libraries(bmp_test hardware math)
add_executable(arduino_test
    ${CMAKE_CURRENT_LIST_DIR}/test/arduino_test.cpp)
target_link_libraries(arduino_test hardware math)
add_executable(pwm_test
    ${CMAKE_CURRENT_LIST_DIR}/test/pwm_test.cpp)
target_link_libraries(pwm_test hardware math)
add_executable(sensor_test
    ${CMAKE_CURRENT_LIST_DIR}/test/sensor_test.cpp)
target_link_libraries(sensor_test hardware math)
