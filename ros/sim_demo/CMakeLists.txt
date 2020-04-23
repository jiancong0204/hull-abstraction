cmake_minimum_required(VERSION 2.8.3)
project(sim_demo)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    template_nodes
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES dummy_package_lib
#   CATKIN_DEPENDS roscpp
#   DEPENDS system_lib
# )

catkin_package(
    CATKIN_DEPENDS
        template_nodes
)





#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opc_ua_1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()


if (CATKIN_ENABLE_TESTING)
    ## Add folder for testing launchfiles
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)
endif()
