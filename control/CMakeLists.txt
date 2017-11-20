include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(control
    ${CMAKE_CURRENT_LIST_DIR}/src/controller.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/state.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/param.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/raw_data.cpp)

add_executable(launch
    ${CMAKE_CURRENT_LIST_DIR}/src/launch.cpp)
target_link_libraries(launch control algorithm logging hardware math)

add_executable(bin2txt
    ${CMAKE_CURRENT_LIST_DIR}/test/bin2txt.cpp)
target_link_libraries(bin2txt control logging math)

add_executable(skips
    ${CMAKE_CURRENT_LIST_DIR}/test/skips.cpp)
target_link_libraries(skips control logging math)
