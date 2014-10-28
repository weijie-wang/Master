cmake_minimum_required (VERSION 2.6)
project (driver)
SET(CMAKE_BUILD_TYPE Debug)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

include_directories("/include")
link_directories("/lib")
add_executable(app timer.cpp recat.cpp app.cpp parse.cpp)
target_link_libraries(app rt ethercat)
