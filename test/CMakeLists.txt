add_executable(unit_tests
    ${CMAKE_CURRENT_LIST_DIR}/src/math.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/logging.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/test_main.cpp)
target_link_libraries(unit_tests logging math)
