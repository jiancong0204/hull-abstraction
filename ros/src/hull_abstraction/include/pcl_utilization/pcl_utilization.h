#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

namespace pcl_utilization
{
    /** Function that converts pcl::PolygonMesh to visualization_msgs::Marker 
    */
    visualization_msgs::Marker toMarkerMsg(pcl::PolygonMesh mesh);
};
