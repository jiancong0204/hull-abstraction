/**
  * @file pcl_utilization.h
  * @brief This file contains the declaration of PCL utilization methods
  *
  * @author Jiancong Zheng
  * @date 2020-05-31
  **/

#pragma once
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl_utilization
{
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
     * @param cloud_with_normals Cloud of PointNormal
     * @return Resolution of the input cloud
     **/
    double computeCloudResolution(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);

    /**
     * @brief Compute the AABB for the input point cloud
     * 
     * @param cloud Cloud of PointXYZ
     * @return A list of the maximal as well as the minimal x-, y- and z-coordinates
     */
    std::vector<std::vector<double>> computeAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief Compute the AABB for the input point cloud
     * 
     * @param cloud_with_normals Cloud of PointNormal
     * @return A list of the maximal as well as the minimal x-, y- and z-coordinates
     */
    std::vector<std::vector<double>> computeAABB(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);
}
