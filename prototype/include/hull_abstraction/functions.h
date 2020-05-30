#pragma once
#include<math.h>
#include <time.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl_utilization
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

namespace benchmark
{
    /* Function that divides the original cloud. */
    void divideCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud, double fraction);
    
    /* Function that divides the original cloud. */
    void divideCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr test_cloud, double fraction);

    /* Intersection between a line and a polygon. */
    std::vector<double> intersectWith(pcl::PolygonMesh mesh, std::vector<double> point, std::vector<double> normal);

    /* Comfirm that a point is inside the polygon. */
    bool isInside(std::vector<double> point, std::vector<std::vector<double>> polygon);
}
