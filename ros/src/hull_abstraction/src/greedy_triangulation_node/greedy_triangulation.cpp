#include "greedy_triangulation.h"

void greedy_triangulation_node::GreedyTriangulation::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*input_msg, *cloud); // Convert ROS message to a cloud
    pp.appendNormalEstimation(cloud, cloud_with_normals); // Add normal estimation to the original cloud
    mesh = rc.greedyTriangulation(cloud_with_normals); // Generate mesh through greedy triangulation algorithm
    output_msg = pcl_utilization::toMarkerMsg(mesh); // Create Marker message for publishing
    pub = nh.advertise<visualization_msgs::Marker>("greedy_triangulation", 1);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void greedy_triangulation_node::GreedyTriangulation::run()
{
    sub = nh.subscribe("load_pcd", 1, &GreedyTriangulation::processing, this);
}