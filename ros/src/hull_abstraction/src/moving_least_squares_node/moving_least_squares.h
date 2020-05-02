#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


namespace moving_least_squares_node {
    class MovingLeastSquares {

    public:
        MovingLeastSquares() {};
        void run();
    
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        sensor_msgs::PointCloud2 output_msg;
        void processing(const sensor_msgs::PointCloud2ConstPtr input_msg);
    };
}
