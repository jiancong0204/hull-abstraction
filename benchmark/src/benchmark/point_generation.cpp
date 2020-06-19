#include "benchmark/point_generation.h"

void benchmark::PointGenerator::inputPolygonMesh(pcl::PolygonMesh mesh)
{
    this->input_mesh = mesh;
}

pcl::PointCloud<pcl::PointNormal>::Ptr benchmark::PointGenerator::getPointCloud()
{
    return this->output_cloud;
}

void benchmark::setSampleSize(size_t size)
{
    this->sample_size = size;
}

void benchmark::PointGenerator::generatePointCloud()
{
    this->output_cloud = randomlySampling(this->input_mesh);
}

pcl::PointCloud<pcl::PointNormal>::Ptr benchmark::PointGenerator::randomlySampling(pcl::PolygonMesh mesh)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud); // Extract point cloud from the input mesh

    std::vector<double> area_mapping; // List for area mapping
    double mesh_area = 0;
    area_mapping.push_back(mesh_area);
    size_t triangle_size = mesh.polygons.size();
    for (size_t i = 0; i < triangle_size; i++)
    {
        mesh_area += benchmark::calculateArea(mesh.polygons[i]);
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

        std::vector<double> sample_point = benchmark::uniformTriangleSampling(triangle);

        pcl::PointNormal point;
        point.x = sample_point[0];
        point.y = sample_point[0];
        point.z = sample_point[0];
        point.normal_x = 1;
        point.normal_y = 1;
        point.normal_z = 1;
        point.normal[0] = 1;
        point.normal[1] = 1;
        point.normal[2] = 1;
        cloud->push_back(point);
    }
    return cloud;
}
