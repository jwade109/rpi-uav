include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(algorithm OBJECT
    ${CMAKE_CURRENT_LIST_DIR}/src/pid.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/filters.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gps_baro_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gps_position_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mass_estimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motor_director.cpp)
