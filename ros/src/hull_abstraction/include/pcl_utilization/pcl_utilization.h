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
     * @brief Convert a polygon mesh to a marker message with a triangle list
     * 
     * @param mesh Triangle mesh
     * @return Marker message as a triangle list
     **/
    visualization_msgs::Marker toTriangleList(pcl::PolygonMesh mesh);

    /**
     * @brief  Convert a polygon mesh to a marker message with a line list
     * 
     * @param mesh Polygon mesh
     * @return Marker message as a line list 
     */
    visualization_msgs::Marker toLineList(pcl::PolygonMesh mesh);

    /**
     * @brief Computing the resolution of a point cloud
     * 
     * @param cloud Cloud of PointXYZ
     * @return Resolution of the input cloud
     **/
    double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Computing the resolution of a point cloud with normals
     * 
     * @param cloud Cloud of PointNormal
     * @return Resolution of the input cloud
     **/
    double computeCloudResolution(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
    
    /**
     * @brief  Calculate the centroid of a point cloud
     * 
     * @param cloud The input point cloud data
     * @return The coordinates of the centroid 
     */
    std::vector<double> computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Calculate the centroid of a mesh
     * 
     * @param mesh The input polygon mesh
     * @return The coordinates of the centroid 
     */
    std::vector<double> computeCentroid(pcl::PolygonMesh mesh);
};
