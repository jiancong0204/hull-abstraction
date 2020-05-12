/**
  * @file
  * @brief This file contains the declaration of PCL utilization methods
  *
  * @author Jiancong Zheng
  * @date 2020-05-12
  **/

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
    /**
     * @brief Convert a polygon mesh to a marker message for triangle lists
     * 
     * @param mesh Triangle mesh
     * @return Marker message as triangle lists
     **/
    visualization_msgs::Marker toMarkerMsg(pcl::PolygonMesh mesh);

    /**
     * @brief Compute the resolution of a point cloud
     * 
     * @param cloud Cloud of PointXYZ
     * @return Resolution of the input cloud
     **/
    double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Compute the resolution of a point cloud with normals
     * 
     * @param cloud Cloud of PointNormal
     * @return Resolution of the input cloud
     **/
    double computeCloudResolution(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
};
