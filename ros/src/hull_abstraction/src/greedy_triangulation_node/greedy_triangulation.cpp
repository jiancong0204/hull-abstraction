#include "greedy_triangulation.h"

void greedy_triangulation::GreedyTriangulation::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*input_msg, *cloud); // Convert ROS message to a cloud
    pp.appendNormalEstimation(cloud, cloud_with_normals); // Add normal estimation to the original cloud
    mesh = rc.greedyTriangulation(cloud_with_normals); // Generate mesh through greedy triangulation algorithm
    pcl_conversions::fromPCL(mesh, output_msg); // Convert a polygon mesh to a mesh message
    
    pub = nh.advertise<pcl_msgs::PolygonMesh>("greedy_triangulation", 10);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void greedy_triangulation::GreedyTriangulation::run()
{
    sub = nh.subscribe("load_pcd", 1, &GreedyTriangulation::processing, this);
}