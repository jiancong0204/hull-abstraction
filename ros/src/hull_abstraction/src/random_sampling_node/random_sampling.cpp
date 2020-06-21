#include "random_sampling.h"

void random_sampling::RandomSampling::processing(const pcl_msgs::PolygonMesh input_msg)
{
    // Convert from message to mesh in pcl
    pcl::PolygonMesh mesh;
    pcl_conversions::toPCL(this->input_msg, mesh);

    // Perform random sampling
    pg.inputPolygonMesh(mesh);
    pg.setSampleSize(1000);
    pg.generatePointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = pg.getPointCloud();

    // Generate message for publishing
    pcl::toROSMsg(*cloud, this->output_msg);
    this->output_msg.header.frame_id = "base";

    // Publish message
    this->pub = this->nh.advertise<sensor_msgs::PointCloud2>("random_sampling", 1);
    this->pub.publish(this->output_msg);
}


void random_sampling::RandomSampling::run()
{
    sub = nh.subscribe("poisson_reconstruction", 1, &RandomSampling::processing, this);
}