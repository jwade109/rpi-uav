include_directories("${CMAKE_CURRENT_LIST_DIR}/kalman/")

add_executable(kalman ${CMAKE_CURRENT_LIST_DIR}/kalman/main.cpp)
target_link_libraries(kalman uav)
add_executable(freebody ${CMAKE_CURRENT_LIST_DIR}/kalman/free.cpp)
target_link_libraries(freebody uav)
