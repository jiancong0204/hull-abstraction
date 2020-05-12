#include "pcl_utilization/pcl_utilization.h"

visualization_msgs::Marker pcl_utilization::toMarkerMsg(pcl::PolygonMesh mesh)
{
    visualization_msgs::Marker marker;
    int polygon_size;
    int vertices_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
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
    color.r = 1;
    color.g = 1;
    color.b = 1;

    Eigen::Vector4f centroid;
    polygon_size = mesh.polygons.size();
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
    pcl::compute3DCentroid(*mesh_cloud,centroid); // Estimate the coordinates of the centroid.
    for (int i = 0; i < polygon_size; i++)
    {
        std::vector<geometry_msgs::Point> temp_point(3);
        for (int j = 0; j < 3; j++)
        {
            temp_point[j].x = mesh_cloud->points[mesh.polygons[i].vertices[j]].x;
            temp_point[j].y = mesh_cloud->points[mesh.polygons[i].vertices[j]].y;
            temp_point[j].z = mesh_cloud->points[mesh.polygons[i].vertices[j]].z;
        }
        std::vector<double> temp_centroid(3);

        // Calculate the center of the triangle.
        temp_centroid[0] = (temp_point[0].x + temp_point[1].x + temp_point[2].x) / 3;
        temp_centroid[1] = (temp_point[0].y + temp_point[1].y + temp_point[2].y) / 3;
        temp_centroid[2] = (temp_point[0].z + temp_point[1].z + temp_point[2].z) / 3;

        // Vector from centriod to temp_centroid.
        double temp_vector_x = temp_centroid[0]; //- centroid[0];
        double temp_vector_y = temp_centroid[1]; //- centroid[1];
        double temp_vector_z = temp_centroid[2]; //- centroid[2];

        double x_1 = temp_point[1].x - temp_point[0].x;
        double x_2 = temp_point[2].x - temp_point[1].x;
        double y_1 = temp_point[1].y - temp_point[0].y;
        double y_2 = temp_point[2].y - temp_point[1].y;
        double z_1 = temp_point[1].z - temp_point[0].z;
        double z_2 = temp_point[2].z - temp_point[1].z;

        // Normal vector of the triangle.
        double temp_n_x = y_1 * z_2 - z_1 * y_2;
        double temp_n_y = z_1 * x_2 - x_1 * z_2;
        double temp_n_z = x_1 * y_2 - y_1 * x_2;

        // Dor product
        double dot_product = temp_vector_x * temp_n_x + temp_vector_y * temp_n_y + temp_vector_z * temp_n_z;

        if (dot_product < 0)
        {
            marker.points.push_back(temp_point[0]);
            marker.points.push_back(temp_point[1]);
            marker.points.push_back(temp_point[2]);       
        }
        else 
        {
            marker.points.push_back(temp_point[0]);
            marker.points.push_back(temp_point[2]);
            marker.points.push_back(temp_point[1]);           
        }
        marker.color = color;
    }
    return marker;
}

double pcl_utilization::computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!std::isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    // std::cout << "number of point: " << numberOfPoints << std::endl;
    return resolution;
}

double pcl_utilization::computeCloudResolutionN(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(cloudWithNormals);

    for (size_t i = 0; i < cloudWithNormals->size(); ++i)
    {
        if (!std::isfinite((*cloudWithNormals)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    // std::cout << "number of point: " <<numberOfPoints << std::endl;
    return resolution;
}