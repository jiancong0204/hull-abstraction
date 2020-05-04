#include "poisson_reconstruction.h"

void poisson_reconstruction::PoissonReconstruction::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::fromROSMsg(*input_msg, *cloud);
        pp.appendNormalEstimation(cloud, cloud_with_normals);
    mesh = rc.poissonReconstruction(cloud_with_normals);
    pub = nh.advertise<pcl_msgs::PolygonMesh>("poisson_reconstruction", 1);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void poisson_reconstruction::PoissonReconstruction::run()
{
    sub = nh.subscribe("load_pcd", 1, &PoissonReconstruction::processing, this);
}