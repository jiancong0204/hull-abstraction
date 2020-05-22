#pragma once
#include <pcl/search/kdtree.h>

namespace hull_abstraction
{
    /* Function that calculates resolution of point cloud. */
    double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /* Function that calculates resolution of point cloud with normal estimation. */
    double computeCloudResolution(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);

    /* Function that calculates the coordinates of AABB. */
    std::vector<std::vector<double>> computeAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /* Function that calculates the coordinates of AABB. */
    std::vector<std::vector<double>> computeAABB(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);
}
