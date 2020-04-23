cmake_minimum_required(VERSION 2.8.3)
project(template_nodes)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Remove ROS_DEBUG from executable for minimal release builds
if ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    # ROS packages
    roscpp
    std_srvs
    dynamic_reconfigure
    # IGMR packages
    igmr_node_interface
    template_msgs
    template_srvs
    template_actions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS regex)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/DoSomething.cfg
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
    INCLUDE_DIRS
        include
    LIBRARIES
        example_library
        threading_library
    CATKIN_DEPENDS
        std_srvs
        dynamic_reconfigure
        template_msgs
        template_srvs
        template_actions
    DEPENDS
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/example.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME}
##    Depends on targets from this package
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
##    Depends on targets from packages found via find_package()
#     ${catkin_EXPORTED_TARGETS}
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_example_node
#     src/example_node.cpp
# )

## Create executable without PROJECT_NAME prefix
## The above recommended prefix causes long target names, the following renames the
## executable to the shorter version for ease of use
## e.g. "rosrun template_pkg example_node"
## instead of "rosrun template_pkg template_pkg_example_node"
# set_target_properties(${PROJECT_NAME}_example_node
#     PROPERTIES OUTPUT_NAME example_node PREFIX ""
# )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_example_node
##    Depends on targets from this package
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
##    Depends on targets from packages found via find_package()
#     ${catkin_EXPORTED_TARGETS}
# )

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_example_node
#     ${catkin_LIBRARIES}
# )



# example_library library
add_library(${PROJECT_NAME}_example_library
    src/example_library/example_class.cpp
)
target_include_directories(${PROJECT_NAME}_example_library
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_example_library
    PROPERTIES
        OUTPUT_NAME "example_library"
#        PREFIX      "lib"
)

# threading_library library
add_library(${PROJECT_NAME}_threading_library
    src/threading_library/waitthread.cpp
)
target_include_directories(${PROJECT_NAME}_threading_library
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_threading_library
    PROPERTIES
        OUTPUT_NAME "threading_library"
#        PREFIX      "lib"
)


# do_something node
add_executable(${PROJECT_NAME}_do_something_node
    src/do_something_node.cpp
    src/do_something_node/do_something.cpp
)
target_include_directories(${PROJECT_NAME}_do_something_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_do_something_node
    PROPERTIES
        OUTPUT_NAME "do_something"
)
target_link_libraries(${PROJECT_NAME}_do_something_node
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_do_something_node
    ${catkin_EXPORTED_TARGETS}
)


# do_something_big node
add_executable(${PROJECT_NAME}_do_something_big_node
   src/do_something_big_node.cpp
   src/do_something_big_node/actions.cpp
   src/do_something_big_node/do_something_big.cpp
   src/do_something_big_node/dynamic_reconfigure.cpp
   src/do_something_big_node/publications.cpp
   src/do_something_big_node/services.cpp
   src/do_something_big_node/subscriptions.cpp
)
target_include_directories(${PROJECT_NAME}_do_something_big_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_do_something_big_node
    PROPERTIES
        OUTPUT_NAME "do_something_big"
)
target_link_libraries(${PROJECT_NAME}_do_something_big_node
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_do_something_big_node
    ${catkin_EXPORTED_TARGETS}
)


# do_something_action node
add_executable(${PROJECT_NAME}_do_something_action_node
    src/do_something_action_node.cpp
    src/do_something_action_node/do_something_action.cpp
)
target_include_directories(${PROJECT_NAME}_do_something_action_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_do_something_action_node
    PROPERTIES
        OUTPUT_NAME "do_something_action"
)
target_link_libraries(${PROJECT_NAME}_do_something_action_node
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_do_something_action_node
    ${catkin_EXPORTED_TARGETS}
)


# async_service node
add_executable(${PROJECT_NAME}_async_service_node
    src/async_service_node.cpp
    src/async_service_node/async_service.cpp
)
target_include_directories(${PROJECT_NAME}_async_service_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_async_service_node
    PROPERTIES
        OUTPUT_NAME "async_service"
)
target_link_libraries(${PROJECT_NAME}_async_service_node
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_async_service_node
    ${catkin_EXPORTED_TARGETS}
)


# threading node
add_executable(${PROJECT_NAME}_threading_node
    src/threading_node.cpp
    src/threading_node/threading.cpp
    src/threading_node/server/exampleserver.cpp
)
target_include_directories(${PROJECT_NAME}_threading_node
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_threading_node
    PROPERTIES
        OUTPUT_NAME "threading"
)
target_link_libraries(${PROJECT_NAME}_threading_node
    ${PROJECT_NAME}_threading_library
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_threading_node
    ${catkin_EXPORTED_TARGETS}
)


# benchmarks
add_executable(${PROJECT_NAME}_benchmarks
    benchmark/${PROJECT_NAME}/benchmarks.cpp
)
target_include_directories(${PROJECT_NAME}_benchmarks
    PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)
set_target_properties(${PROJECT_NAME}_benchmarks
    PROPERTIES
        OUTPUT_NAME "benchmarks"
)
target_link_libraries(${PROJECT_NAME}_benchmarks
    benchmark ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opc_ua_1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()


if (CATKIN_ENABLE_TESTING)
    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)

    ## Add gtest based test targets
    find_package(GTest REQUIRED)

    catkin_add_gtest(${PROJECT_NAME}_conduct_heavy_computation_test
        test/example_library/conduct_heavy_computation_test.cpp
    )

    set_target_properties(${PROJECT_NAME}_conduct_heavy_computation_test PROPERTIES OUTPUT_NAME "conduct_heavy_computation_test")

    target_link_libraries(${PROJECT_NAME}_conduct_heavy_computation_test
        ${PROJECT_NAME}_example_library
        ${catkin_LIBRARIES}
    )

    catkin_add_gtest(${PROJECT_NAME}_template_class_test
        test/example_templates/template_class_test.cpp
    )

    set_target_properties(${PROJECT_NAME}_template_class_test PROPERTIES OUTPUT_NAME "template_class_test")

    target_link_libraries(${PROJECT_NAME}_template_class_test
        #${PROJECT_NAME}_example_templates
        ${catkin_LIBRARIES}
    )

    ## Add rostest based test target
    # find_package(rostest REQUIRED)
    # add_rostest(tests/your_first_rostest.test)

    ## Add test for launch files
    find_package(roslaunch REQUIRED)
    # roslaunch_add_file_check(launch)
endif()
