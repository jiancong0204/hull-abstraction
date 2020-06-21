#include "load_pcd.h"

void load_pcd::LoadPCD::run()
{
    this->pub = this->nh.advertise<sensor_msgs::PointCloud2> ("load_pcd", 10);
    createROSMsg();
    publishROSMsg();
}

bool load_pcd::LoadPCD::createROSMsg()
{
    bool is_loaded = ~pcl::io::loadPCDFile ("/home/jc/THESIS/hull_abstraction/benchmark/point_cloud_data/16_5.pcd", cloud);
    if (is_loaded)
        std::cout << "PCD file loaded. " << std::endl;
    else
    {
        std::cout << "Loading failed! " << std::endl;
        return false;
    }
    pcl::toROSMsg(this->cloud, this->output_msg);
    this->output_msg.header.frame_id = "base";
    return true;
}

void load_pcd::LoadPCD::publishROSMsg() {
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        this->output_msg.header.stamp = ros::Time::now();
        this->pub.publish(this->output_msg);
        ros::spinOnce();
        std::cout << "Cloud is published. " << std::endl;
        loop_rate.sleep();
    }
}
