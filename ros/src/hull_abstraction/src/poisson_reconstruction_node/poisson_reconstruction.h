/**
 * @file poisson_reconstruction.h
 *
 * @brief Framework of Poisson reconstruction node
 * @ros_node poisson_reconstruction
 * 
 * @author Jiancong Zheng
 * @date 2020-05-12
 * 
 * This node subscribes a ROS topic to get an input point cloud and then utilize Poisson reconstruction to generate a mesh.
 * The result of Poisson reconstruction is published as a point cloud message.
 */
#pragma once
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include "hull_abstraction/reconstructor.h"
#include "hull_abstraction/preprocessor.h"

namespace poisson_reconstruction
{
    /**
     * @brief Class utilizing Poisson reconstruction method
     * 
     * This framework is developed to apply Poisson reconstruction to generate mesh based on the input point cloud.
     */
    class PoissonReconstruction
    {
    public:
        /**
         * @brief Construct a new PoissonReconstruction object
         */
        PoissonReconstruction() {}

        /**
         * @brief Encapsulate a method to run the poisson_reconstruction node
         */
        void run();

    private:
        ros::NodeHandle                   nh;           /**< Node Handle reference from embedding node */
        ros::Publisher                    pub;          /**< Polygon mesh publisher */
        ros::Subscriber                   sub;          /**< Raw point cloud subscriber */
        pcl_msgs::PolygonMesh             output_msg;   /**< Polygon mesh message used to publish the mesh */

        hull_abstraction::Reconstructor   rc;           /**< Object for Reconstructor class */
        hull_abstraction::Preprocessor    pp;           /**< Object for Preprocessor class */
        pcl::PolygonMesh                  mesh;         /**< Resulted polygon mesh */

        /**
         * @brief Process the point cloud message to create mesh message 
         * 
         * @param input_msg Input point cloud message
         */
        void processing(const sensor_msgs::PointCloud2ConstPtr input_msg);

    };
}