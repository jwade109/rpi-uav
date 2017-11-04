set(CMAKE_CXX_STANDARD 11)

include_directories("${CMAKE_CURRENT_LIST_DIR}/kalman/")
include_directories("/usr/local/include/eigen3")

add_executable(kalman ${CMAKE_CURRENT_LIST_DIR}/kalman/main.cpp)
target_link_libraries(kalman uav)
