cmake_minimum_required(VERSION 3.10)
project(region_analysis)

find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(analyzer main.cpp)
target_link_libraries(analyzer ${PCL_LIBRARIES} ${Boost_LIBRARIES})
