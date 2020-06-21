#include "point_generation/point_generation.h"

void point_generation::PointGenerator::inputPolygonMesh(pcl::PolygonMesh mesh)
{
    this->input_mesh = mesh;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr point_generation::PointGenerator::getPointCloud()
{
    return this->output_cloud;
}

void point_generation::PointGenerator::setSampleSize(size_t size)
{
    this->sample_size = size;
}

void point_generation::PointGenerator::generatePointCloud()
{
    this->output_cloud = randomlySampling(this->input_mesh);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr point_generation::PointGenerator::randomlySampling(pcl::PolygonMesh mesh)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud); // Extract point cloud from the input mesh

    std::vector<double> area_mapping; // List for area mapping
    double mesh_area = 0;
    area_mapping.push_back(mesh_area);
    size_t triangle_size = mesh.polygons.size();
    for (size_t i = 0; i < triangle_size; i++)
    {
        std::vector<std::vector<double>> triangle; 
        for (size_t j = 0; j < 3; j++)
        {
            triangle.push_back({
                mesh_cloud->points[mesh.polygons[i].vertices[j]].x, 
                mesh_cloud->points[mesh.polygons[i].vertices[j]].y,
                mesh_cloud->points[mesh.polygons[i].vertices[j]].z
            });
        }
        mesh_area += pcl_utilization::calculateArea(triangle);
        area_mapping.push_back(mesh_area);
    }

    srand(time(NULL));

    for (int i = 0; i < this->sample_size; i++)
    {
        double random_area = rand()%1001;
        random_area = random_area / 1000 * mesh_area;
        int lower_bound = 0;
        int upper_bound = triangle_size - 1;
        while (true)
        {
            if (upper_bound - lower_bound == 1) break;
            int middle = (upper_bound + lower_bound) / 2;
            if (random_area < area_mapping[middle]) 
            {
                upper_bound = middle;
            }
            else 
            {
                lower_bound = middle;
            }
        }

        std::vector<std::vector<double>> triangle; 
        for (size_t j = 0; j < 3; j++)
        {
            triangle.push_back({
                mesh_cloud->points[mesh.polygons[lower_bound].vertices[j]].x, 
                mesh_cloud->points[mesh.polygons[lower_bound].vertices[j]].y,
                mesh_cloud->points[mesh.polygons[lower_bound].vertices[j]].z
            });
        }

        std::vector<double> sample_point = pcl_utilization::uniformTriangleSampling(triangle);

        pcl::PointXYZ point;
        point.x = sample_point[0];
        point.y = sample_point[1];
        point.z = sample_point[2];

        // std::vector<double> normal_vector = pcl_utilization::calculateNormal(triangle);
        // point.normal_x = normal_vector[0];
        // point.normal_y = normal_vector[1];
        // point.normal_z = normal_vector[2];
        // point.normal[0] = normal_vector[0];
        // point.normal[1] = normal_vector[1];
        // point.normal[2] = normal_vector[2];
        cloud->push_back(point);
    }
    return cloud;
}
