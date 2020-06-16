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
        intersection_point = benchmark::intersectWith(this->mesh, point, normal);
        double distance = intersection_point[3]; // distance between test point and the mesh
        oFile << fabs(distance) << std::endl;
    }
}