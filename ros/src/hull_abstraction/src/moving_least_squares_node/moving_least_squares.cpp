#include "moving_least_squares.h"

void moving_least_squares_node::MovingLeastSquares::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudFilteredWithNormals (new pcl::PointCloud<pcl::PointNormal>);

    pcl::fromROSMsg(*input_msg, *cloud); // Convert point cloud message to a cloud
    pp.movingLeastSquares(cloud, cloudFiltered, cloudFilteredWithNormals); // Perform MLS method
    pcl::toROSMsg(*cloudFiltered, output_msg); // Convert the cloud to a point cloud message
    pub = nh.advertise<sensor_msgs::PointCloud2>("moving_least_squares", 1);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void moving_least_squares_node::MovingLeastSquares::run()
{
    sub = nh.subscribe("load_pcd", 1, &MovingLeastSquares::processing, this);
}
