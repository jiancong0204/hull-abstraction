cmake_minimum_required(VERSION 2.8.3)
project(template_actions)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    actionlib
    actionlib_msgs
)


################################################
############## Declare ROS actions #############
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
## * In this file (CMakeLists.txt):
##   * add every package in MSG_DEP_SET to find_package(catkin REQUIRED COMPONENTS ...)
##   * add every package in MSG_DEP_SET to catkin_package(CATKIN_DEPENDS ...)
##   * list every .msg/.srv/.action file to be processed in the add_action_files section
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    DoSomething.action
)


## Generate added messages with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    actionlib
    actionlib_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)
