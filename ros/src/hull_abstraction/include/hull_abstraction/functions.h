#pragma once
#include <pcl/search/kdtree.h>

namespace hull_abstraction
{
    /* Function that calculates resolution of point cloud. */
    double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /* Function that calculates resolution of point cloud with normal estimation. */
    double computeCloudResolutionN(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);
}
