cmake_minimum_required(VERSION 2.8.3)
project(vibration_navigator_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
)

add_service_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
