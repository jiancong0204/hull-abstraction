#include "load_pcd.h"

void load_pcd::LoadPCD::run(int argc, char **argv) {
    initialization(argc, argv);
    createROSMsg();
    publish();
}

void load_pcd::LoadPCD::initialization(int argc, char **argv) {
    ros::init (argc, argv, "load_pcd"); // Node name
    ros::NodeHandle nh;
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("load_pcd", 1);
}

void load_pcd::LoadPCD::createROSMsg() {
    //Convert the cloud to ROS message
    pcl::io::loadPCDFile ("/home/jc/master_thesis/prototype/point_cloud_data/16_5.pcd", input_cloud);
    pcl::toROSMsg(input_cloud, output_cloud);
    output_cloud.header.frame_id = "base";
}

void load_pcd::LoadPCD::publish() {
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output_cloud);
        ros::spinOnce();
        std::cout << "published" << std::endl;
        loop_rate.sleep();
    }

}

