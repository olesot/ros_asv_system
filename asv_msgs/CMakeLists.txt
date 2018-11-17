cmake_minimum_required(VERSION 2.8.3)
project(asv_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
)

add_message_files(
  FILES
  ShipMetaData.msg
  State.msg
  StateArray.msg
  Offset.msg
)

add_service_files(
  FILES
  Intersect.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


