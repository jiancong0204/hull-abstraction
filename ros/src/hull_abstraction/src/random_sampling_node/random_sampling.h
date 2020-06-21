/**
 * @file random_sampling.h
 * @author Jiancong Zheng
 * @brief Framework of Random Sampling node
 * @date 2020-06-21
 * This node subscribes a ROS topic to get an input triangle mesh and then performs random sampling on the mesh to generate a point cloud. 
 */

#pragma once
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_utilization/math_computing.h"
#include "point_generation/point_generation.h"
