cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(surfaces)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (generate_points generate_points.cpp)
target_link_libraries (generate_points ${PCL_LIBRARIES})
  
#add_executable(proccess_surface proccess_surface.cpp)
#target_link_libraries(proccess_surface ${PCL_LIBRARIES})
