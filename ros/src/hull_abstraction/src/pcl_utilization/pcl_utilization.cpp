#include "pcl_utilization/pcl_utilization.h"

visualization_msgs::Marker pcl_utilization::toLineList(pcl::PolygonMesh mesh)
{
    visualization_msgs::Marker marker;
    int polygon_size;
    int vertices_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std_msgs::ColorRGBA color;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "base";

    // Marker parameters
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    color.a = 1;  // Transparency
    color.r = 1;
    color.g = 1;
    color.b = 1;

    polygon_size = mesh.polygons.size();
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    for (int i = 0; i < polygon_size; i++)
    {
        vertices_size = mesh.polygons[i].vertices.size();
        geometry_msgs::Point temp_point;
        for (int j = 0; j < vertices_size - 1; j++)
        {
            temp_point.x = mesh_cloud->points[mesh.polygons[i].vertices[j]].x;
            temp_point.y = mesh_cloud->points[mesh.polygons[i].vertices[j]].y;
            temp_point.z = mesh_cloud->points[mesh.polygons[i].vertices[j]].z;
            marker.points.push_back(temp_point);
            temp_point.x = mesh_cloud->points[mesh.polygons[i].vertices[j + 1]].x;
            temp_point.y = mesh_cloud->points[mesh.polygons[i].vertices[j + 1]].y;
            temp_point.z = mesh_cloud->points[mesh.polygons[i].vertices[j + 1]].z;
            marker.points.push_back(temp_point);
        }
    }
    marker.color = color;
    return marker;
}

visualization_msgs::Marker pcl_utilization::toTriangleList(pcl::PolygonMesh mesh)
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
        vertices_size = mesh.polygons[i].vertices.size();
        std::vector<geometry_msgs::Point> temp_point(vertices_size);
        for (int j = 0; j < vertices_size; j++)
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
        double temp_vector_z = temp_centroid[2]; //- centroid[2];pcl::PolygonMesh mesh

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
            numberOfPoints++;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    // std::cout << "number of point: " << numberOfPoints << std::endl;
    return resolution;
}

double pcl_utilization::computeCloudResolution(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(cloud_with_normals);

    for (size_t i = 0; i < cloud_with_normals->size(); ++i)
    {
        if (!std::isfinite((*cloud_with_normals)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            numberOfPoints++;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
    // std::cout << "number of point: " <<numberOfPoints << std::endl;
    return resolution;
}

std::vector<double> pcl_utilization::computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Eigen::Vector4f centroid;
    std::vector<double> cloud_centroid(3);
    
    pcl::compute3DCentroid(*cloud, centroid);
    for (int i = 0; i < 3; i++)
    {
        cloud_centroid[i] = centroid[i];
    }
    return cloud_centroid;
}

std::vector<double> pcl_utilization::computeCentroid(pcl::PolygonMesh mesh)
{
    int polygon_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<double> mesh_centroid = {0.0, 0.0, 0.0};

    polygon_size = mesh.polygons.size();
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    for (int i = 0; i < polygon_size; i++)
    {
        int vertices_size = mesh.polygons[i].vertices.size();
        std::vector<double> polygon_centroid = {0.0, 0.0, 0.0};
        for (int j = 0; j < vertices_size - 1; j++)
        {
            polygon_centroid[0] += mesh_cloud->points[mesh.polygons[i].vertices[j]].x;
            polygon_centroid[1] += mesh_cloud->points[mesh.polygons[i].vertices[j]].y;
            polygon_centroid[2] += mesh_cloud->points[mesh.polygons[i].vertices[j]].z;
        }

        // vector<int>::iterator itr=polygon_centroid.begin();
        polygon_centroid[0] /= vertices_size;
        polygon_centroid[1] /= vertices_size;
        polygon_centroid[2] /= vertices_size;
        
        mesh_centroid[0] += polygon_centroid[0];
        mesh_centroid[1] += polygon_centroid[1];
        mesh_centroid[2] += polygon_centroid[2];
    }
        mesh_centroid[0] /= polygon_size;
        mesh_centroid[1] /= polygon_size;
        mesh_centroid[2] /= polygon_size;
        return mesh_centroid;
}

std::vector<std::vector<double>> pcl_utilization::computeAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int cloud_size = cloud->points.size(); // Number of the points
    std::vector<std::vector<double>> result;
    std::vector<double> max;
    std::vector<double> min;

    max.push_back(cloud->points[0].x);
    max.push_back(cloud->points[0].y);
    max.push_back(cloud->points[0].z);
    min.push_back(cloud->points[0].x);
    min.push_back(cloud->points[0].y);
    min.push_back(cloud->points[0].z);

    for (int i = 1; i < cloud_size; i++)
    {
        if (cloud->points[i].x > max[0])
        {
            max[0] = cloud->points[i].x;
        }
        else if (cloud->points[i].x < min[0])
        {
            min[0] = cloud->points[i].x;
        }
        else;

        if (cloud->points[i].y > max[1])
        {
            max[1] = cloud->points[i].y;
        }
        else if (cloud->points[i].y < min[1])
        {
            min[1] = cloud->points[i].y;
        }
        else;


        if (cloud->points[i].z > max[2])
        {
            max[2] = cloud->points[i].z;
        }
        else if (cloud->points[i].z < min[2])
        {
            min[2] = cloud->points[i].z;
        }
        else;
    }

    result.push_back(max);
    result.push_back(min);
    return result;
}
