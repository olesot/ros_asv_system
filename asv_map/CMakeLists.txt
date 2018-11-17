cmake_minimum_required(VERSION 2.8.3)
project(asv_map)

find_library(gdal_LIBRARIES
  gdal
)

find_package(catkin REQUIRED
  roscpp
  rospy
  roslib
  asv_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES map_service
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs asv_msgs roscpp rospy roslib
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}
)

add_library(asv_map_service
  src/asv_map_service_node.cpp
)

add_executable(asv_map_service_node
  src/asv_map_service_node.cpp
  src/asv_map_service.cpp
  src/vincent.cpp
)

add_dependencies(asv_map_service_node
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(asv_map_service_node
  ${catkin_LIBRARIES}
  ${gdal_LIBRARIES}
)


