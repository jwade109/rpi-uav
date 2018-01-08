include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(algorithm
    ${CMAKE_CURRENT_LIST_DIR}/src/pid.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/filters.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/gps_baro_filter.cpp
    # ${CMAKE_CURRENT_LIST_DIR}/src/gps_position_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mass_estimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motor_director.cpp)

add_executable(ins_test ${CMAKE_CURRENT_LIST_DIR}/test/ins_test.cpp)
target_link_libraries(ins_test hardware algorithm math)

# add_executable(filter_test
#     ${CMAKE_CURRENT_LIST_DIR}/test/filter_test.cpp)
# target_link_libraries(filter_test algorithm hardware math)

# add_executable(pid_test
#     ${CMAKE_CURRENT_LIST_DIR}/test/pid_test.cpp)
# target_link_libraries(pid_test algorithm math)
