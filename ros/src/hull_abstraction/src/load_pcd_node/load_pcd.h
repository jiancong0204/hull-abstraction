#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

namespace load_pcd
{
    class LoadPCD
    {
    public:
        LoadPCD() {}
        ~LoadPCD() {}
        void run();

    private:
        ros::NodeHandle nh;
        ros::Publisher pcl_pub;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        sensor_msgs::PointCloud2 msg;
        int createROSMsg();
        void publishROSMsg();
    };
}

