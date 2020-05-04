#pragma once
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include "hull_abstraction/reconstructor.h"
#include "hull_abstraction/preprocessor.h"

namespace greedy_triangulation
{
    class GreedyTriangulation
    {
    public:
        GreedyTriangulation() {}
        void run();

    private:
        hull_abstraction::Reconstructor rc;
        hull_abstraction::Preprocessor pp;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals {new pcl::PointCloud<pcl::PointNormal>};
        pcl::PolygonMesh mesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        void processing(const sensor_msgs::PointCloud2ConstPtr input_msg);
        pcl_msgs::PolygonMesh output_msg;
    };
}
