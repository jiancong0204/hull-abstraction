#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>

namespace load_pcd {
    class LoadPCD {
    public:
        LoadPCD() {}
        ~LoadPCD() {}
        void run(int argc, char **argv);

    private:
        ros::Publisher pcl_pub;
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        sensor_msgs::PointCloud2 output_cloud;
        void initialization(int argc, char **argv);
	void createROSMsg();
        void publish();
    };
}

