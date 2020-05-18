#include "bspline_surface_fitting.h"

void bspline_surface_fitting_node::BsplineSurfaceFitting::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*input_msg, *cloud); // Convert ROS message to a cloud
    mesh = rc.bsplineSurfaceFitting(cloud); // Generate mesh through b-spline surface fitting
    pcl_conversions::fromPCL(mesh, output_msg); // Convert a polygon mesh to a mesh message
    
    pub = nh.advertise<pcl_msgs::PolygonMesh>("bspline_surface_fitting", 10); // Topic for publishing
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void bspline_surface_fitting_node::BsplineSurfaceFitting::run()
{
    sub = nh.subscribe("load_pcd", 1, &BsplineSurfaceFitting::processing, this); // Subscribe the topic of "load_pcd"
}