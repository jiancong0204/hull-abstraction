/**
 * @file random_sampling.h
 * @brief Framework of Random Sampling node
 * 
 * @ros_node random_sampling_node
 * 
 * @author Jiancong Zheng
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

namespace random_sampling
{
    /**
     * @brief Class that performs random sampling
     * This framework is developed to perform random sampling on a triangle mesh to generate point cloud data.
     */
    class RandomSampling
    {

    public:

        /**
         * @brief Construct a new RandomSampling object
         * 
         */
        RandomSampling() {};

        /**
         * @brief Destroy the RandomSampling object
         * 
         */
        ~RandomSampling() {};

        /**
         * @brief Encapsulate a method to run the random_sampling node
         */
        void run();

    private:

        ros::NodeHandle                      nh;            /**< Node Handle reference from embedding node */
        ros::Publisher                       pub;           /**< Point cloud publisher */
        ros::Subscriber                      sub;           /**< Raw triangle mesh subscriber */
        sensor_msgs::PointCloud2             output_msg;    /**< point cloud message used to publish the result */
        point_generation::PointGenerator     pg;            /**< PointGenerator object */

        /**
         * @brief Process the mesh message to create point cloud message 
         * 
         * @param input_msg Input point cloud message
         */
        void processing(const pcl_msgs::PolygonMesh input_msg);

    };
}