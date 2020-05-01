cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(hull_abstraction)

find_package(PCL 1.10 REQUIRED) # find PCL library and define the required version
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_msgs
    rospy
    sensor_msgs
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hull_abstraction
#  CATKIN_DEPENDS pcl_conversions pcl_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

# Load pcd flie
SET(load_pcd 
    src/load_pcd_node.cpp 
    src/load_pcd_node/load_pcd.cpp
)

add_executable(load_pcd ${load_pcd})

target_link_libraries(load_pcd
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Moving least squares method
SET(moving_least_squares 
    src/moving_least_squares_node.cpp 
    src/moving_least_squares_node/moving_least_squares.cpp
    src/hull_abstraction/functions.cpp
)

add_executable(moving_least_squares ${moving_least_squares})

target_link_libraries(moving_least_squares
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)
