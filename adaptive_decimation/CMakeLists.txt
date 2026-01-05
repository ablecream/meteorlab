cmake_minimum_required(VERSION 3.10)
project(CurvatureBasedDecimation)

# Find PCL
find_package(PCL 1.10 REQUIRED)
find_package(VTK REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(curvature_decimator main.cpp)

target_link_libraries(curvature_decimator
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

# For newer versions of VTK (>8), enable modules
if (VTK_VERSION VERSION_GREATER "8")
  vtk_module_autoinit(
    TARGETS curvature_decimator
    MODULES ${VTK_LIBRARIES}
  )
endif()
