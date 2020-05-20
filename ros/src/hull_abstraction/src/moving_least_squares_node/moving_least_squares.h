/**
 * @file moving_least_squares.h
 * 
 * @brief Framework of moving least squares node
 * @ros_node moving_least_squares_node
 * 
 * @author Jiancong Zheng
 * @date 2020-05-12
 * 
 * This node subscribes a ROS topic to get an input point cloud and then utilize moving least squares method to make the point cloud smoother.
 * MLS method can be seen as a preprocessing method.
 * The result of moving least squares method is published as a point cloud message.
 */

#pragma once
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "hull_abstraction/preprocessor.h"

namespace moving_least_squares_node
{
    /**
     * @brief Class utilizing moving least squares method
     * 
     * This framework is developed to apply moving least squares method to smoothen the input point cloud.
     */
    class MovingLeastSquares
    {
    public:
        /**
         * @brief Construct a new MovingLeastSquares object
         */
        MovingLeastSquares() {};

        /**
         * @brief Encapsulate a method to run the moving_least_squares node
         */
        void run();
    
    private:

        ros::NodeHandle                       nh;           /**< Node Handle reference from embedding node */
        ros::Publisher                        pub;          /**< Point cloud publisher */
        ros::Subscriber                       sub;          /**< Raw point cloud subscriber */
        sensor_msgs::PointCloud2              output_msg;   /**< point cloud message used to publish the result */

        hull_abstraction::Preprocessor        pp;           /**< Object for Preprocessor class */

        /**
         * @brief Processing the input ROS message
         * 
         * @param input_msg Input ROS message
         */
        void processing(const sensor_msgs::PointCloud2ConstPtr input_msg);
    };
}

