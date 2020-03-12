cmake_minimum_required(VERSION 2.8.3)
project(vibration_navigator_driver)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    std_msgs
    roscpp
    rospy
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(Boost REQUIRED
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        std_msgs
        roscpp
        rospy
        tf2
        tf2_ros
        tf2_geometry_msgs
    DEPENDS
        Boost
        EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

#
add_executable(
    vibration_navigator_driver_node
    src/vibration_navigator_driver_node.cpp
    src/vibration_navigator_driver.cpp
)
target_link_libraries(
    vibration_navigator_driver_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)