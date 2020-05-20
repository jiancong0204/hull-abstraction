/**
 * @file greedy_triangulation.h
 * 
 * @brief Framework of greedy triangulation node
 * @ros_node greedy_triangulation_node
 * 
 * @author Jiancong Zheng
 * @date 2020-05-12
 * 
 * This node subscribes a ROS topic to get an input point cloud and then utilize greedy triangulation algorithm to generate a mesh.
 * The result of greedy triangulation is published as a Marker message with triangle lists.
 */

#pragma once

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include "hull_abstraction/reconstructor.h"
#include "hull_abstraction/preprocessor.h"
#include "pcl_utilization/pcl_utilization.h"

namespace greedy_triangulation
{
    /**
     * @brief Class utilizing greedy triangulation method
     * 
     * This framework is developed to generate a mesh for a point cloud through greedy triangulation.
     */
    class GreedyTriangulation
    {
    public:
        /**
         * @brief Construct a new GreedyTriangulation object
         */
        GreedyTriangulation() {}

        /**
         * @brief  Encapsulate a method to run the greedy_triangulation node
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
