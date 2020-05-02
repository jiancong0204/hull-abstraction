#include "hull_abstraction/functions.h"

double hull_abstraction::computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!std::isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    // std::cout << "number of point: " << numberOfPoints << std::endl;
    return resolution;
}

double hull_abstraction::computeCloudResolutionN(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(cloudWithNormals);

    for (size_t i = 0; i < cloudWithNormals->size(); ++i)
    {
        if (!std::isfinite((*cloudWithNormals)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    std::cout << "number of point: " <<numberOfPoints << std::endl;
    return resolution;
}
