cmake_minimum_required(VERSION 3.5)

project(hull_abstraction)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(PCL 1.7 REQUIRED) # find PCL library and define the required version
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_msgs
    roscpp
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

# Display mesh
SET(display_mesh 
    src/display_mesh_node.cpp 
    src/display_mesh_node/display_mesh.cpp
    src/pcl_utilization/pcl_utilization.cpp
)

add_executable(display_mesh ${display_mesh})

target_link_libraries(display_mesh
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Moving least squares method
SET(moving_least_squares 
    src/moving_least_squares_node.cpp 
    src/moving_least_squares_node/moving_least_squares.cpp
    src/pcl_utilization/pcl_utilization.cpp
    src/hull_abstraction/preprocessor.cpp
)

add_executable(moving_least_squares ${moving_least_squares})

target_link_libraries(moving_least_squares
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# B-spline surface fitting method
SET(bspline_surface_fitting 
    src/bspline_surface_fitting_node.cpp 
    src/bspline_surface_fitting_node/bspline_surface_fitting.cpp
    src/hull_abstraction/reconstructor.cpp
    src/pcl_utilization/pcl_utilization.cpp
)

add_executable(bspline_surface_fitting ${bspline_surface_fitting})

target_link_libraries(bspline_surface_fitting
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Greedy triangulation algorithm
SET(greedy_triangulation 
    src/greedy_triangulation_node.cpp 
    src/greedy_triangulation_node/greedy_triangulation.cpp
    src/hull_abstraction/reconstructor.cpp
    src/hull_abstraction/preprocessor.cpp
    src/pcl_utilization/pcl_utilization.cpp
)

add_executable(greedy_triangulation ${greedy_triangulation})

target_link_libraries(greedy_triangulation
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Poisson reconstruction algorithm
SET(poisson_reconstruction 
    src/poisson_reconstruction_node.cpp 
    src/poisson_reconstruction_node/poisson_reconstruction.cpp
    src/pcl_utilization/pcl_utilization.cpp
    src/hull_abstraction/reconstructor.cpp
    src/hull_abstraction/preprocessor.cpp
)

add_executable(poisson_reconstruction ${poisson_reconstruction})

target_link_libraries(poisson_reconstruction
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Marching cubes algorithm
SET(marching_cubes 
    src/marching_cubes_node.cpp 
    src/marching_cubes_node/marching_cubes.cpp
    src/hull_abstraction/reconstructor.cpp
    src/hull_abstraction/preprocessor.cpp
    src/pcl_utilization/pcl_utilization.cpp
    src/pcl_utilization/math_computing.cpp
    src/point_generation/point_generation.cpp
)

add_executable(marching_cubes ${marching_cubes})

target_link_libraries(marching_cubes
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Random Sampling
SET(random_sampling
    src/pcl_utilization/math_computing.cpp
)
