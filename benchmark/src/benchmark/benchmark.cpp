#include "benchmark/benchmark.h"

void benchmark::Benchmark::inputPolygonMesh(pcl::PolygonMesh mesh)
{
    this->mesh = mesh;
}

void benchmark::Benchmark::inputPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
    this->input_cloud = cloud;
    benchmark::Benchmark::divideCloud(this->input_cloud, this->test_cloud, this->fraction);
}

pcl::PointCloud<pcl::PointNormal>::Ptr benchmark::Benchmark::getTestCloud()
{
    return this->test_cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr benchmark::Benchmark::getInputCloud()
{
    return this->input_cloud;
}

void benchmark::Benchmark::setTestCloudSize(double fraction)
{
    this->fraction = fraction;
}

void benchmark::Benchmark::divideCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr test_cloud, double fraction)
{
    int cloud_size = cloud->points.size(); // Number of the points
    int test_size = floor(fraction * cloud_size);
    test_cloud->height = cloud->height;
    test_cloud->height = cloud->width;
    test_cloud->points.resize(test_size);

    srand((unsigned)time(NULL));
    for (int i = 0; i < test_size; i++)
    {
        cloud_size = cloud->points.size();
        // pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
        float random_number = rand() % cloud_size;
        test_cloud->points[i].x = cloud->points[random_number].x;
        test_cloud->points[i].y = cloud->points[random_number].y;
        test_cloud->points[i].z = cloud->points[random_number].z;
        test_cloud->points[i].normal_x = cloud->points[random_number].normal_x;
        test_cloud->points[i].normal_y = cloud->points[random_number].normal_y;
        test_cloud->points[i].normal_z = cloud->points[random_number].normal_z;
        test_cloud->points[i].normal[0] = cloud->points[random_number].normal[0];
        test_cloud->points[i].normal[1] = cloud->points[random_number].normal[1];
        test_cloud->points[i].normal[2] = cloud->points[random_number].normal[2];
        cloud->erase(cloud->begin() + random_number);
    }
}

std::vector<double> benchmark::Benchmark::intersectWith(pcl::PolygonMesh mesh, std::vector<double> point, std::vector<double> normal)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    double x0 = point[0];
    double y0 = point[1];
    double z0 = point[2];
    double t = INFINITY;
    size_t polygon_size = mesh.polygons.size();
    std::vector<double> intersection_point = {0.0, 0.0, 0.0, 1.0};

    for (size_t i = 0; i < polygon_size; i++)
    {
        // Parameters for the plane which contains the polygon
        std::vector<double> x = {0.0, 1.0, 0.0};
        std::vector<double> y = {0.0, 0.0, 1.0};
        std::vector<double> z = {1.0, 1.0, 1.0};
        std::vector<std::vector<double>> polygon;
        
        int vertices_size = mesh.polygons[i].vertices.size();
        for (int j = 0; j < vertices_size; j++)
        {
            if (j < 3)
            {
                x[j] = mesh_cloud->points[mesh.polygons[i].vertices[j]].x;
                y[j] = mesh_cloud->points[mesh.polygons[i].vertices[j]].y;
                z[j] = mesh_cloud->points[mesh.polygons[i].vertices[j]].z;
            }
            polygon.push_back({
                mesh_cloud->points[mesh.polygons[i].vertices[j]].x, 
                mesh_cloud->points[mesh.polygons[i].vertices[j]].y,
                mesh_cloud->points[mesh.polygons[i].vertices[j]].z
                });
        }

        double a = (y[1] - y[0]) * (z[2] - z[0]) - (y[2] - y[0]) * (z[1] -z[0]);
        double b = (z[1] - z[0]) * (x[2] - x[0]) - (z[2] - z[0]) * (x[1] -x[0]);
        double c = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] -y[0]);

        // Parameter for intersection point
        double tmp = ((x[0] - x0) * a + (y[0] - y0) * b + (z[0] - z0) * c) / (a * normal[0] + b * normal[1] + c * normal[2]);
        intersection_point[0] = x0 + normal[0] * tmp;
        intersection_point[1] = y0 + normal[1] * tmp;
        intersection_point[2] = z0 + normal[2] * tmp; 

        bool is_inside = benchmark::Benchmark::isInside(intersection_point, polygon);

        if ((fabs(tmp) < fabs(t)) and is_inside)
        {
            t = tmp;
        }
        else; // Do nothing

    }
    if (t == INFINITY)
    {
        intersection_point[0] = x0 + normal[0];
        intersection_point[1] = y0 + normal[1];
        intersection_point[2] = z0 + normal[2]; 
    }
    else
    {
        intersection_point[0] = x0 + normal[0] * t;
        intersection_point[1] = y0 + normal[1] * t;
        intersection_point[2] = z0 + normal[2] * t; 
        intersection_point[3] = t;
    }
    return intersection_point;
}

bool benchmark::Benchmark::isInside(std::vector<double> point, std::vector<std::vector<double>> polygon)
{
    std::vector<double> angles;
    for (auto itr = polygon.begin(); itr != polygon.end(); ++itr)
    {
        std::vector<double> vector1;
        std::vector<double> vector2;

        if ((itr+1) != polygon.end())
        {
            vector1.push_back((*itr)[0] - point[0]);
            vector1.push_back((*itr)[1] - point[1]);
            vector1.push_back((*itr)[2] - point[2]);
            itr++;
            vector2.push_back((*itr)[0] - point[0]);
            vector2.push_back((*itr)[1] - point[1]);
            vector2.push_back((*itr)[2] - point[2]);
            itr--;
        }
        else
        {
            vector1.push_back((*itr)[0] - point[0]);
            vector1.push_back((*itr)[1] - point[1]);
            vector1.push_back((*itr)[2] - point[2]);
            itr = polygon.begin();
            vector2.push_back((*itr)[0] - point[0]);
            vector2.push_back((*itr)[1] - point[1]);
            vector2.push_back((*itr)[2] - point[2]);
            itr = polygon.end();
            itr--;
        }

        double length1 = sqrt(pow(vector1[0], 2) + pow(vector1[1], 2) + pow(vector1[2], 2)); 
        double length2 = sqrt(pow(vector2[0], 2) + pow(vector2[1], 2) + pow(vector2[2], 2));
        if (length1 * length2 == 0)
            return true;
        double dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
        angles.push_back(acos(dot_product / (length1 * length2)));
    }

    double angle_sum = 0.0;
    for(auto itr = angles.begin(); itr != angles.end(); itr++)
    {
        angle_sum += (*itr);
    }
    // std::cout << angle_sum << std::endl;
    // std::cout << angle_sum / M_PI << std::endl;
    double angle_sum_rad = angle_sum / M_PI;
    if(angle_sum_rad > 1.999 and angle_sum_rad < 2.001)  // allow tiny errors
        return true;
    else 
        return false;
}

void benchmark::Benchmark::generateData(std::string file_name)
{
    int test_cloud_size = this->test_cloud->points.size();
    ofstream oFile;
    oFile.open("../test_data/" + file_name + ".csv", ios::out | ios::trunc);
    for (int i = 0; i < test_cloud_size; i++)
    {
        std::vector<double> point;
        point.push_back(this->test_cloud->points[i].x);
        point.push_back(this->test_cloud->points[i].y);
        point.push_back(this->test_cloud->points[i].z);

        std::vector<double> normal;
        normal.push_back(this->test_cloud->points[i].normal_x);
        normal.push_back(this->test_cloud->points[i].normal_y);
        normal.push_back(this->test_cloud->points[i].normal_z);

        std::vector<double> intersection_point;
        intersection_point = benchmark::Benchmark::intersectWith(this->mesh, point, normal);
        double distance = intersection_point[3]; // distance between test point and the mesh
        oFile << fabs(distance) << std::endl;
    }
}