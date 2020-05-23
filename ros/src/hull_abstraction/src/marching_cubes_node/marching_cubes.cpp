#include "marching_cubes.h"

void marching_cubes::MarchingCubes::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Generate the mesh
    pcl::fromROSMsg(*input_msg, *cloud); // Convert ROS message to a cloud
    pp.appendNormalEstimation(cloud, cloud_with_normals); // Add normal estimation to the original cloud
    mesh = rc.marchingCubes(cloud_with_normals); // Generate mesh through marching cubes algorithm
    
    // Calculate the centroid of the cloud
    std::vector<double> cloud_centroid = pcl_utilization::computeCentroid(cloud);
    std::cout << "The centroid of the cloud: [" << cloud_centroid[0] << ", " << cloud_centroid[1] << ", " << cloud_centroid[2] << "] " << std::endl;
    
    // Calculate the centroid of the mesh
    std::vector<double> mesh_centroid = pcl_utilization::computeCentroid(mesh);
    std::cout << "The centroid of the mesh:  [" << mesh_centroid[0] << ", " << mesh_centroid[1] << ", " << mesh_centroid[2] << "] " << std::endl;
    std::cout << std::endl;
    
    // Publish the mesh
    pcl_conversions::fromPCL(mesh, output_msg); // Convert a polygon mesh to a mesh message
    pub = nh.advertise<pcl_msgs::PolygonMesh>("marching_cubes", 10);
    pub.publish(output_msg);
}

void marching_cubes::MarchingCubes::run()
{
    sub = nh.subscribe("load_pcd", 1, &MarchingCubes::processing, this);
}
