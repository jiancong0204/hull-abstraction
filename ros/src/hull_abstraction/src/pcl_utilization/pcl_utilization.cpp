#include "pcl_utilization/pcl_utilization.h"

visualization_msgs::Marker pcl_utilization::toMarkerMsg(pcl::PolygonMesh mesh)
{
    visualization_msgs::Marker marker;
    int polygon_size;
    int vertices_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::Point temp_point;
    std_msgs::ColorRGBA color;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "base";
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    color.a = 1;
    color.r = 255;
    color.g = 255;
    color.b = 255;
    polygon_size = mesh.polygons.size();
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
    for (int i = 0; i < polygon_size; i++)
    {
        vertices_size = mesh.polygons[i].vertices.size();
        for (int j = 0; j < vertices_size; j++)
        {
            temp_point.x = mesh_cloud->points[mesh.polygons[i].vertices[j]].x;
            temp_point.y = mesh_cloud->points[mesh.polygons[i].vertices[j]].y;
            temp_point.z = mesh_cloud->points[mesh.polygons[i].vertices[j]].z;
            marker.points.push_back(temp_point);
            marker.color = color;
        }
    }
    return marker;
}
