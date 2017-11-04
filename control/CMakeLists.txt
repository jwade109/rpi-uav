include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(control OBJECT
    ${CMAKE_CURRENT_LIST_DIR}/src/controller.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/state.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/param.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/raw_data.cpp)

add_executable(launch
    ${CMAKE_CURRENT_LIST_DIR}/src/launch.cpp)
target_link_libraries(launch uav)
