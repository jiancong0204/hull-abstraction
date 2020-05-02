#include "moving_least_squares.h"

void moving_least_squares_node::MovingLeastSquares::processing(const sensor_msgs::PointCloud2ConstPtr input_msg)
{
    pcl::fromROSMsg(*input_msg, *cloud); 
    pp.movingLeastSquares(cloud, cloudFiltered, cloudFilteredWithNormals);
    pcl::toROSMsg(*cloudFiltered, output_msg);
    pub = nh.advertise<sensor_msgs::PointCloud2>("moving_least_squares", 1);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void moving_least_squares_node::MovingLeastSquares::run()
{
    sub = nh.subscribe("load_pcd", 1, &MovingLeastSquares::processing, this);
}
