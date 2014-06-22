cmake_minimum_required(VERSION 2.8.3)
project(galileo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs	
  std_msgs
  geometry_msgs  
  tf
)

set(ROS_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#rosbuild_genmsg()

add_message_files(FILES
  Skeleton.msg
  SkeletonJoint.msg
  Skeletons.msg
)

generate_messages(DEPENDENCIES 
  geometry_msgs std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES galileo
  DEPENDS system_lib
  CATKIN_DEPENDS 
    cv_bridge 
    image_transport 
    roscpp 
    sensor_msgs 
    std_msgs
    geometry_msgs
    tf
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${OpenNi_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIR}
  ${OpenNi_INCLUDE_DIR}
  ${Nite_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)

# find Orocos kdl
find_package(orocos_kdl REQUIRED)

# find Openni
find_package(PkgConfig)
include(FindPkgConfig)
pkg_check_modules(OpenNi REQUIRED libopenni)

## Declare a cpp executable
add_executable(capture_image_node src/capture_image_node.cpp)
target_link_libraries(capture_image_node
   ${catkin_LIBRARIES}
)

add_executable(capture_image_node2 src/capture_image_node2.cpp)
target_link_libraries(capture_image_node2
   ${catkin_LIBRARIES}
)

include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)
add_executable(listener_transform_node src/listener_transform.cpp)
target_link_libraries(listener_transform_node ${catkin_LIBRARIES} ${OPENNI_LIBRARIES})

include_directories(${OPENNI_INCLUDEDIR} /usr/include/nite)
add_executable(skeleton_angles_node src/skeleton_angles.cpp)
target_link_libraries(skeleton_angles_node 
  ${catkin_LIBRARIES} 
  ${OpenNI_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

#
#add_executable(kinect_angles_node src/kinect_angles.cpp)
#target_link_libraries(kinect_angles_node
#   ${catkin_LIBRARIES}
#)

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

