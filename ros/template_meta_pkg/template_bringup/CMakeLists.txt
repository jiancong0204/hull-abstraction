cmake_minimum_required(VERSION 2.8.3)
project(template_bringup)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package()


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS roslaunch)
    roslaunch_add_file_check(launch
        DEPENDENCIES
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
endif()
