cmake_minimum_required(VERSION 2.8.3)
project(galileo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)

set(ROS_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES galileo
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf
  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(capture_image
#   src/capture_image_node.cpp
#)

## Declare a cpp executable
add_executable(capture_image_node src/capture_image_node.cpp)
target_link_libraries(capture_image_node
   ${catkin_LIBRARIES}
)

add_executable(capture_image_node2 src/capture_image_node2.cpp)
target_link_libraries(capture_image_node2
   ${catkin_LIBRARIES}
)

add_executable(listener_transform_node src/listener_transform.cpp)
target_link_libraries(listener_transform_node
   ${catkin_LIBRARIES}
)

add_executable(kinect_angles_node src/kinect_angles.cpp)
target_link_libraries(kinect_angles_node
   ${catkin_LIBRARIES}
)

install(TARGETS capture_image_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
