/**
 * @file load_pcd.h
 * 
 * @brief Framework of node for loading pcd files
 * 
 * @author Jiancong Zheng 
 * @date 2020-05-12
 * 
 * This node load a local pcd file to get point cloud data and publish it as a ROS message.
 */
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

namespace load_pcd
{
    /**
     * @brief Class for loading pcd files
     */
    class LoadPCD
    {
    public:
        /**
         * @brief Construct a new LoadPCD object
         */
        LoadPCD() {}

        /**
         * @brief Destroy the LoadPCD object
         */
        ~LoadPCD() {}

        /**
         * @brief Encapsulate a method to run the load_pcd node
         */
        void run();

    private:
        ros::NodeHandle                   nh;           /**< Node Handle reference from embedding node */
        ros::Publisher                    pub;          /**< Point cloud publisher */
        sensor_msgs::PointCloud2          output_msg;   /**< point cloud message used to publish the result */
        pcl::PointCloud<pcl::PointXYZ>    cloud;        /**< Cloud storing the point cloud loaded from the pcd file */

        /**
         * @brief Create a ROS message for the point cloud
         * 
         * @return true when the pcd file is successfully loaded
         * @return false when the pcd file is not loaded
         */
        bool createROSMsg();

        /**
         * @brief Publish the message
         * 
         */
        void publishROSMsg();
    };
}

