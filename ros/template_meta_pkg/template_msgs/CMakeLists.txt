cmake_minimum_required(VERSION 2.8.3)
project(template_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
#    std_msgs
)


################################################
############# Declare ROS messages #############
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages (e.g. std_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
## * In this file (CMakeLists.txt):
##   * add every package in MSG_DEP_SET to find_package(catkin REQUIRED COMPONENTS ...)
##   * add every package in MSG_DEP_SET to catkin_package(CATKIN_DEPENDS ...)
##   * add every .msg file to be processed to the add_message_files section
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Example.msg
    Example2.msg
    DoSomething.msg
)


## Generate added messages with any dependencies listed here
generate_messages(
    DEPENDENCIES
#    std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
    CATKIN_DEPENDS
    message_runtime
#    std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)
