#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "hull_abstraction/reconstructor.h"
#include "hull_abstraction/preprocessor.h"
#include "benchmark/benchmark.h"

int main()
{
    clock_t start, end;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr      test_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr      input_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr         filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr      cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr      filtered_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PolygonMesh                            mesh1;
    pcl::PolygonMesh                            mesh2;
    pcl::PolygonMesh                            mesh3;
    pcl::PolygonMesh                            mesh4;
    hull_abstraction::Preprocessor              pp;
    hull_abstraction::Reconstructor             rc;
    benchmark::Benchmark                        bm;

    //  Load original cloud and check if the file exists
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../point_cloud_data/16_5.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file.\n");
        return(-1);
    }

    // std::cout << CLOCKS_PER_SEC << std::endl;

    // Down sampling
    // start = clock();
    // pp.statisticalFilter(cloud0, cloud);
    // end = clock();
    // std::cout << "Time cost for down sampling: " << (end - start) << " μs" << std::endl;
    
    for (int i = 0; i < 1; i ++)
    {
    // Moving least squares
    // start = clock();
    pp.movingLeastSquares(cloud, filtered_cloud, filtered_cloud_with_normals);
    // end = clock();
    // std::cout << "Time cost for moving least squares algorithm: " << (end - start) << " μs" << std::endl;

    // Normal estimation
    // start = clock();
    pp.appendNormalEstimation(filtered_cloud, cloud_with_normals);
    // end = clock();
    // std::cout << "Time cost for normal estimation: " << (end - start)  << " μs" << std::endl;

    // Generate test cloud and input cloud
    bm.setTestCloudSize(0.05);
    bm.inputPointCloud(cloud_with_normals);
    test_cloud = bm.getTestCloud();
    input_cloud = bm.getInputCloud();

    // int input_cloud_size = input_cloud->points.size();
    // int test_cloud_size = test_cloud->points.size();
    // std::cout << "size of input cloud: " << input_cloud_size << std::endl;
    // std::cout << "size of test cloud: " << test_cloud_size << std::endl;

    start = clock();
    mesh1 = rc.greedyTriangulation(input_cloud);
    end = clock();
    std::cout << "Time cost for greedy triangulation algorithm: " << (end - start)  << " μs" << std::endl;

    // bm.inputPolygonMesh(mesh1);
    // std::string tmp_string = "16_5_15%/Greedy Triangulation/" + std::to_string(i);
    // char tmp_char_array[tmp_string.length()];
    // std::copy(tmp_string.begin(), tmp_string.end(), tmp_char_array);
    // bm.generateData(tmp_char_array);

    start = clock();
    mesh3 = rc.poissonReconstruction(input_cloud);
    end = clock();
    std::cout << "Time cost for Poisson reconstruction: " << (end - start)  << " μs" << std::endl;

    // bm.inputPolygonMesh(mesh3);
    // std::string tmp_string = "16_5_15%/Poisson Reconstruction/" + std::to_string(i);
    // bm.generateData(tmp_string);

    start = clock();
    mesh4 = rc.marchingCubes(input_cloud);
    end = clock();
    std::cout << "Time cost for marching cubes algorithm: " << (end - start)  << " μs" << std::endl;

    // bm.inputPolygonMesh(mesh4);
    // std::string tmp_string = "16_5_5%/Marching Cubes/" + std::to_string(i);
    // bm.generateData(tmp_string);

    pcl::PointCloud<pcl::PointXYZ>::Ptr b_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud, *b_input_cloud);
    start = clock();
    mesh2 = rc.bsplineSurfaceFitting(b_input_cloud);
    end = clock();
    std::cout << "Time cost for b-spline surface fitting: " << (end - start)  << " μs" << std::endl;

    //bm.inputPolygonMesh(mesh2);
    //std::string tmp_string = "16_5_5%/B-spline Surface Fitting/" + std::to_string(i);
    //bm.generateData(tmp_string);

    }

    // Test of function intersectWith()
    // std::vector<double> point = {test_cloud->points[0].x, test_cloud->points[0].y, test_cloud->points[0].z};
    // std::vector<double> normal = {test_cloud->points[0].normal_x, test_cloud->points[0].normal_y, test_cloud->points[0].normal_z};
    // std::vector<double> intersection_point(4);

    // intersection_point = benchmark::intersectWith(mesh3, point, normal);
    // std::cout << intersection_point[0] << std::endl;
    // std::cout << intersection_point[1] << std::endl;
    // std::cout << intersection_point[2] << std::endl;
    // std::cout << intersection_point[3] << std::endl;
    
    // Test of function isInside()
    // std::vector<double> point = {0.4562, 0.3565, 0.0};
    // std::vector<std::vector<double>> polygon = {
    //     {0.0, 1.0, 0.0},
    //     {-1.0, 0.0, 0.0},
    //     {0.0, -1.0, 0.0},
    //     {1.0, 0.0, 0.0}
    // };
    // bool flag = benchmark::isInside(point, polygon);
    // std::cout << flag << std::endl;

    // Display clouds
    // Create a window for visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0), v2(0), v3(0), v4(0), v0(0), v5(0), v6(0);
    viewer->createViewPort(0.00, 0.75, 1.00, 1.00, v0);
    viewer->createViewPort(0.00, 0.50, 0.50, 0.75, v1);
    viewer->createViewPort(0.50, 0.50, 1.00, 0.75, v2);
    viewer->createViewPort(0.00, 0.25, 0.50, 0.50, v3);
    viewer->createViewPort(0.50, 0.25, 1.00, 0.50, v4);
    viewer->createViewPort(0.00, 0.00, 0.50, 0.25, v5);
    viewer->createViewPort(0.50, 0.00, 1.00, 0.25, v6);

    // Set the background
    viewer->setBackgroundColor(0, 0, 0, v0);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->setBackgroundColor(0, 0, 0, v3);
    viewer->setBackgroundColor(0, 0, 0, v4);
    viewer->setBackgroundColor(0, 0, 0, v5);
    viewer->setBackgroundColor(0, 0, 0, v6);

    // Add text
    viewer->addText("Point Cloud", 10, 10, "text0", v0);
    viewer->addText("Greedy Triangulation", 10, 10, "text1", v3);
    viewer->addText("B-Spline Surface Fitting", 10, 10, "text2", v4);
    viewer->addText("Poisson Reconstruction", 10, 10, "text3", v5);
    viewer->addText("Marching Cubes Algorithm", 10, 10, "text4", v6);

    // Add point clouds
    viewer->addPointCloud(cloud, "cloud0", v0);
    viewer->addPointCloud<pcl::PointNormal>(input_cloud, "cloud1", v1);
    viewer->addPointCloud<pcl::PointNormal>(test_cloud, "cloud2", v2);

    // Add meshes
    viewer->addPolygonMesh(mesh1, "mesh1", v3);
    viewer->addPolygonMesh(mesh2, "mesh2", v4);
    viewer->addPolygonMesh(mesh3, "mesh3", v5);
    viewer->addPolygonMesh(mesh4, "mesh4", v6);

    viewer->addPointCloud<pcl::PointNormal>(test_cloud, "cloud5", v5);
    viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(test_cloud, test_cloud, 1, 1.00, "cloud456", v5);

    // viewer->setRepresentationToWireframeForAllActors();
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}
