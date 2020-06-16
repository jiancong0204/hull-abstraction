/**
 * @file math_computing.h
 * @author Jiancong Zheng
 * @brief This file contains declarations of some mathematical computing methods
 * @date 2020-06-16
 * 
 */

#pragma once
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace benchmark
{
    /**
     * @brief Estimate the nearest intersection point between a point's normal vector and the mesh
     * 
     * @param mesh Input mesh representing the surface reconstruction
     * @param point Input Point for estimation
     * @param normal Normal vector of the input point
     * @return A vector containing the coordinates of the resulting intersection point  
     */
    std::vector<double> intersectWith(pcl::PolygonMesh mesh, std::vector<double> point, std::vector<double> normal);

    /**
     * @brief Confirm that a point lies inside a polygon
     * 
     * @param point Input point
     * @param polygon Input polygon
     * @return true The point lies inside the polygon
     * @return false The point lies outside the polygon
     */
    bool isInside(std::vector<double> point, std::vector<std::vector<double>> polygon);

    /**
     * @brief Calculate the area of a triangle
     * 
     * @param Triangle Vertices of the triangle
     * @return Area of the triangle 
     */
    double calculateArea(std::vector<std::vector<double>> triangle);

    /**
     * @brief Calculate the area of a trianlge mesh
     * 
     * @param mesh Input mesh whose area is to be calculated
     * @return Totla area of the triangle mesh
     */
    double calculateArea(pcl::PolygonMesh mesh);
}