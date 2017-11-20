include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")

add_library(math
    ${CMAKE_SOURCE_DIR}/math/src/freebody.cpp
    ${CMAKE_SOURCE_DIR}/math/src/angle.cpp
    ${CMAKE_SOURCE_DIR}/math/src/coordinate.cpp
    ${CMAKE_SOURCE_DIR}/math/src/functions.cpp)
