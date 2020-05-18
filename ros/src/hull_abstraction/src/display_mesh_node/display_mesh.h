/**
 * @file display_mesh.h
 * 
 * @brief Framework of node for displaying polygon meshes.
 * 
 * @author Jiancong Zheng 
 * @date 2020-05-18
 * 
 * This node subscribes topic which contains mseeage for polygon mesh and then process the mesh for display in RVIZ.
 */
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcl_utilization/pcl_utilization.h"

namespace display_mesh
{
    /**
     * @brief  Class for displaying mesh in RVIZ
     */
    class DisplayMesh
    {
    public:
        /**
         * @brief Construct a new Display Mesh object
         */
        DisplayMesh() {};

        /**
         * @brief Destroy the Display Mesh object
         */
        ~DisplayMesh() {};

        /**
         * @brief Encapsulate a method to run the display_mesh node
         * 
         * @param topic Name of topic for input point cloud message
         */
        void run();

    private:
        ros::NodeHandle              nh;           /**< Node Handle reference from embedding node */
        ros::Publisher               pub;          /**< Point cloud publisher */
        ros::Subscriber              sub;          /**< Raw point cloud subscriber */

        visualization_msgs::Marker   output_msg;   /**< Marker message used to publish the mesh */
        /**
         * @brief Process the point cloud message to create mesh message 
         * 
         * @param input_msg Input point cloud message
         */
        void processing(const pcl_msgs::PolygonMesh input_msg);
    };
}