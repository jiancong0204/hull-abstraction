#include "display_mesh.h"

void display_mesh::DisplayMesh::processing(const pcl_msgs::PolygonMesh input_msg)
{
    pcl::PolygonMesh mesh;
    pcl_conversions::toPCL(input_msg, mesh);
    output_msg = pcl_utilization::toLineList(mesh); // Create Marker message for publishing
    pub = nh.advertise<visualization_msgs::Marker>("display_mesh", 10);
    pub.publish(output_msg);
    std::cout << "successfully pulished" << std::endl;
}

void display_mesh::DisplayMesh::run()
{   
    sub = nh.subscribe("poisson_reconstruction", 1, &DisplayMesh::processing, this);
}
