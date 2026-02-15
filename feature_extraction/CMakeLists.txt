cmake_minimum_required(VERSION 3.16)
project(feature_extraction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)

add_executable(feature_extraction main.cpp)

# Link libraries
# Use Boost::headers for header-only libraries like Math
if(TARGET Boost::headers)
    target_link_libraries(feature_extraction PUBLIC CGAL::CGAL Boost::headers)
else()
    target_link_libraries(feature_extraction PUBLIC CGAL::CGAL ${Boost_LIBRARIES})
endif()

# Ensure M_PI is available on all platforms (not just MSVC)
target_compile_definitions(feature_extraction PUBLIC _USE_MATH_DEFINES)