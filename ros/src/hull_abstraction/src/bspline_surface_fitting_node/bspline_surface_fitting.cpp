#include "bspline_surface_fitting.h"

void bspline_surface_fitting_node::BsplineSurfaceFitting::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Generate the mesh
    pcl::fromROSMsg(*input_msg, *cloud); // Convert ROS message to a cloud
    mesh = rc.bsplineSurfaceFitting(cloud); // Generate mesh through b-spline surface fitting
    
    // Calculate the centroid of the cloud
    std::vector<double> cloud_centroid = pcl_utilization::computeCentroid(cloud);
    std::cout << "The centroid of the cloud: [" << cloud_centroid[0] << ", " << cloud_centroid[1] << ", " << cloud_centroid[2] << "] " << std::endl;
    
    // Calculate the centroid of the mesh
    std::vector<double> mesh_centroid = pcl_utilization::computeCentroid(mesh);
    std::cout << "The centroid of the mesh:  [" << mesh_centroid[0] << ", " << mesh_centroid[1] << ", " << mesh_centroid[2] << "] " << std::endl;
    std::cout << std::endl;
    
    // Publish the mesh
    pcl_conversions::fromPCL(mesh, output_msg); // Convert a polygon mesh to a mesh message
    pub = nh.advertise<pcl_msgs::PolygonMesh>("bspline_surface_fitting", 10); // Topic for publishing
    pub.publish(output_msg);
}

void bspline_surface_fitting_node::BsplineSurfaceFitting::run()
{
    sub = nh.subscribe("load_pcd", 1, &BsplineSurfaceFitting::processing, this); // Subscribe the topic of "load_pcd"
}
