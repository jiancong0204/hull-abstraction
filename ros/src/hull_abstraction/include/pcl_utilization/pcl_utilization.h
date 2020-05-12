#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>


namespace pcl_utilization
{
    /** Function that converts pcl::PolygonMesh to visualization_msgs::Marker 
    */
    visualization_msgs::Marker toMarkerMsg(pcl::PolygonMesh mesh);

    /** Function that calculates resolution of point cloud. 
    */
    double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /** Function that calculates resolution of point cloud with normal estimation. 
    */
    double computeCloudResolutionN(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);
};
