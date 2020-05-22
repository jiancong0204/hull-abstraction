#include <iostream>
#include <string>
#include <time.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include "hull_abstraction/reconstructor.h"
#include "hull_abstraction/preprocessor.h"

int main()
{
    clock_t start, end;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PolygonMesh mesh1;
    pcl::PolygonMesh mesh2;
    pcl::PolygonMesh mesh3;
    pcl::PolygonMesh mesh4;
    //  Load original cloud and check if the file exists
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../point_cloud_data/16_5.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file.\n");
        return(-1);
    }

    //std::cout << CLOCKS_PER_SEC << std::endl;
    hull_abstraction::Preprocessor pp;
    hull_abstraction::Reconstructor rc;

    // Down sampling

    //start = clock();
    //pp.statisticalFilter(cloud0, cloud);
    //end = clock();
    //std::cout << "Time cost for down sampling: " << (end - start) << " μs" << std::endl;

    // Moving least squares
    start = clock();
    pp.movingLeastSquares(cloud, filtered_cloud, filtered_cloud_with_normals);
    end = clock();
    std::cout << "Time cost for moving least squares algorithm: " << (end - start) << " μs" << std::endl;

    //Normal estimation
    start = clock();
    pp.appendNormalEstimation(filtered_cloud, cloud_with_normals);
    end = clock();
    std::cout << "Time cost for normal estimation: " << (end - start)  << " μs" << std::endl;

    //start = clock();
    //pp.appendNormalEstimation(filtered_cloud, filtered_cloud_with_normals);
    //end = clock();
    //std::cout << "Time cost for normal estimation (filtered cloud): " << (end - start)  << " μs" << std::endl;

    start = clock();
    mesh1 = rc.greedyTriangulation(cloud_with_normals);
    end = clock();
    std::cout << "Time cost for greedy triangulation algorithm: " << (end - start)  << " μs" << std::endl;

    start = clock();
    mesh2 = rc.bsplineSurfaceFitting(filtered_cloud);
    end = clock();
    std::cout << "Time cost for b-spline surface fitting: " << (end - start)  << " μs" << std::endl;

    start = clock();
    mesh3 = rc.poissonReconstruction(cloud_with_normals);
    end = clock();
    std::cout << "Time cost for Poisson reconstruction: " << (end - start)  << " μs" << std::endl;

    start = clock();
    mesh4 = rc.marchingCubesReconstruction(cloud_with_normals);
    end = clock();
    std::cout << "Time cost for marching cubes algorithm: " << (end - start)  << " μs" << std::endl;


    // Display clouds
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0), v2(0), v3(0), v4(0), v0(0);
    viewer->createViewPort(0.0, 0.66, 1.0, 1.0, v0);
    viewer->createViewPort(0.0, 0.33, 0.5, 0.66, v1);
    viewer->createViewPort(0.5, 0.33, 1.0, 0.66, v2);
    viewer->createViewPort(0.0, 0.0, 0.5, 0.33, v3);
    viewer->createViewPort(0.5, 0.0, 1.0, 0.33, v4);
    
    // Set the background
    viewer->setBackgroundColor(0, 0, 0,v0);
    viewer->setBackgroundColor(0, 0, 0,v1);
    viewer->setBackgroundColor(0, 0, 0,v2);
    viewer->setBackgroundColor(0, 0, 0,v3);
    viewer->setBackgroundColor(0, 0, 0,v4);

    viewer->addText("Point Cloud", 10, 10, "text0", v0);
    viewer->addText("Greedy Triangulation", 10, 10, "text1", v1);
    viewer->addText("B-Spline Surface Fitting", 10, 10, "text2", v2);
    viewer->addText("Poisson Reconstruction", 10, 10, "text3", v3);
    viewer->addText("Marching Cubes Algorithm", 10, 10, "text4", v4);

    viewer->addPointCloud(filtered_cloud, "cloud0", v0);
  
    viewer->addPolygonMesh(mesh1, "mesh1", v1);
    viewer->addPolygonMesh(mesh2, "mesh2", v2);

    viewer->addPolygonMesh(mesh3, "mesh3", v3);
    viewer->addPolygonMesh(mesh4, "mesh4", v4);
    // viewer->setRepresentationToWireframeForAllActors();
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return(0);
}
