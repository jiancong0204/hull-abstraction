#include "load_pcd.h"

void load_pcd::LoadPCD::run()
{
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("load_pcd", 1);
    createROSMsg();
    publishROSMsg();
}

int load_pcd::LoadPCD::createROSMsg()
{
    bool is_loaded = ~pcl::io::loadPCDFile ("/home/jc/hull_abstraction/prototype/point_cloud_data/16_5.pcd", cloud);
    if (is_loaded)
        std::cout << "PCD file loaded. " << std::endl;
    else
    {
        std::cout << "Loading failed! " << std::endl;
        return 0;
    }
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "base";
    return 1;
}

void load_pcd::LoadPCD::publishROSMsg() {
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(msg);
        ros::spinOnce();
        std::cout << "Cloud is published. " << std::endl;
        loop_rate.sleep();
    }
}
