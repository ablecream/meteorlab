cmake_minimum_required(VERSION 3.10)
project(region_detector)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(region_detector main.cpp)
target_link_libraries(region_detector ${PCL_LIBRARIES})


