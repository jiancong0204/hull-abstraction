#include "point_generation/point_generator.h"

void point_generation::PointGenerator::inputPolygonMesh(pcl::PolygonMesh mesh)
{
    this->input_mesh = mesh;
}

pcl::PointCloud<pcl::PointNormal>::Ptr point_generation::PointGenerator::getPointCloud()
{
    return this->output_cloud;
}

void point_generation::PointGenerator::generatePointCloud()
{
    this->output_cloud = randomlySampling(this->input_mesh);
}

pcl::PointCloud<pcl::PointNormal>::Ptr randomlySampling(pcl::PolygonMesh mesh)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    retun cloud;
}
