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
    
    /**
     * @brief Calculate the central symmetry point
     * 
     * @param point Source point
     * @param center_point central point
     * @return Central symmetry point of the source point 
     */
    std::vector<double> calculateCentralSymmetryPoint(std::vector<double> point, std::vector<double> center_point);

    /**
     * @brief Calculate the intersection point between a line going through point 1 and point 2 and its perpendicular line going through point 3
     * 
     * @param point1 Point 1
     * @param point2 Point 2
     * @param point3 Point 3
     * @return Coordinates of the intersection point 
     */
    std::vector<double> calculatePerpendicularIntersection(std::vector<double> point1, std::vector<double> point2, std::vector<double> point3);

    /**
     * @brief Sample a point inside a triangle (srand(time(NULL)) must be executed before calling this function)
     * 
     * @param triangle Trigle on which a point will be sampled
     * @return Coordinates of the point
     */
    std::vector<double> uniformTriangleSampling(std::vector<std::vector<double>> triangle);

    /**
     * @brief Calculate the normal vector of a triangle
     * 
     * @param triangle Input triangle
     * @return double Normal vector
     */
    std::vector<double> calculateNormal(std::vector<std::vector<double>> triangle);
}
