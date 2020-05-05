#include "greedy_triangulation.h"

void greedy_triangulation::GreedyTriangulation::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::fromROSMsg(*input_msg, *cloud);
    pp.appendNormalEstimation(cloud, cloud_with_normals);
    mesh = rc.greedyTriangulation(cloud_with_normals);
    output_msg = pcl_utilization::toMarkerMsg(mesh);
    pub = nh.advertise<visualization_msgs::Marker>("greedy_triangulation", 1);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void greedy_triangulation::GreedyTriangulation::run()
{
    sub = nh.subscribe("load_pcd", 1, &GreedyTriangulation::processing, this);
}