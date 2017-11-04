include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(hardware OBJECT
    ${CMAKE_CURRENT_LIST_DIR}/src/ardimu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/bmp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gps.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/i2c.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pwm.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sensors.cpp)
