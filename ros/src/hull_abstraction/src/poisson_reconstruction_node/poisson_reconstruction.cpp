#include "poisson_reconstruction.h"

void poisson_reconstruction::PoissonReconstruction::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input_msg, *cloud); // Convert point cloud message to a cloud
    pp.appendNormalEstimation(cloud, cloud_with_normals); // Add normal estimation to the original cloud
    mesh = rc.poissonReconstruction(cloud_with_normals); // Perform Poisson reconstruction
    pcl_conversions::fromPCL(mesh, output_msg); // Convert a polygon mesh to a mesh message
    
    pub = nh.advertise<pcl_msgs::PolygonMesh>("poisson_reconstruction", 1);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void poisson_reconstruction::PoissonReconstruction::run()
{
    sub = nh.subscribe("load_pcd", 1, &PoissonReconstruction::processing, this);
}