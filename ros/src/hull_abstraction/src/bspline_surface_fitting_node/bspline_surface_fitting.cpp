#include "bspline_surface_fitting.h"

void bspline_surface_fitting_node::BsplineSurfaceFitting::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud); // Convert ROS message to a cloud
    mesh = rc.bsplineSurfaceFitting(cloud); // Generate mesh through b-spline surface fitting
    output_msg = pcl_utilization::toMarkerMsg(mesh); // Create Marker message for publishing
    pub = nh.advertise<visualization_msgs::Marker>("bspline_surface_fitting", 1); // Topic for publishing
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void bspline_surface_fitting_node::BsplineSurfaceFitting::run()
{
    sub = nh.subscribe("load_pcd", 1, &BsplineSurfaceFitting::processing, this); // Subscribe the topic of "load_pcd"
}