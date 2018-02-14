include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(algorithm
    ${CMAKE_CURRENT_LIST_DIR}/src/pid.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/filters.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gps_baro_filter.cpp
    # ${CMAKE_CURRENT_LIST_DIR}/src/gps_position_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mass_estimator.cpp
    # ${CMAKE_CURRENT_LIST_DIR}/src/motor_director.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/freebody.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/navigator.cpp)

add_executable(nav_test ${CMAKE_CURRENT_LIST_DIR}/test/nav_test.cpp)
target_link_libraries(nav_test hardware algorithm math)

add_executable(motor_ctrl ${CMAKE_CURRENT_LIST_DIR}/src/motor_control.cpp)

add_executable(fbtest ${CMAKE_CURRENT_LIST_DIR}/test/fbtest.cpp)
target_link_libraries(fbtest hardware algorithm math)

# add_executable(filter_test
#     ${CMAKE_CURRENT_LIST_DIR}/test/filter_test.cpp)
# target_link_libraries(filter_test algorithm hardware math)

# add_executable(pid_test
#     ${CMAKE_CURRENT_LIST_DIR}/test/pid_test.cpp)
# target_link_libraries(pid_test algorithm math)
