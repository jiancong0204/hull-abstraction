#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/mls.h>
#include "hull_abstraction/preprocessor.h"
#include "hull_abstraction/functions.h"
#include <pcl/io/pcd_io.h>


#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

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
