#pragma once
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "hull_abstraction/preprocessor.h"

namespace moving_least_squares_node
{
    class MovingLeastSquares
    {

    public:
        MovingLeastSquares() {};
        void run();
    
    private:
        hull_abstraction::Preprocessor pp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudFilteredWithNormals{new pcl::PointCloud<pcl::PointNormal>};
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        sensor_msgs::PointCloud2 output_msg;
        void processing(const sensor_msgs::PointCloud2ConstPtr input_msg);
    };
}

