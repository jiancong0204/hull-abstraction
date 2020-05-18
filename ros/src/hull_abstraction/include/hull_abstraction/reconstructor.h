/**
  * @file
  * @brief This file contains the declaration of the Reconstructor class.
  *
  * @author Jiancong Zheng
  * @date 2020-05-12
  **/

#pragma once

#include "hull_abstraction/preprocessor.h"

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/surface/gp3.h> // Head file for the greedy triangulation method
#include <pcl/surface/poisson.h>  // Head file for Poisson reconstruction method
#include <pcl/surface/marching_cubes_hoppe.h> // Head file for marching cubes method
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h> // Head file for b-spline surface fitting
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h> // Head file for b-spline curve fitting
#include <pcl/surface/on_nurbs/triangulation.h> // Head file for creating triangulation on b-spline surface

namespace hull_abstraction
{
    /**
     * @brief The Reconstructor class
     *
     * This class wraps some hull construction methods for point cloud data
     **/
    class Reconstructor 
    {
    public:
        /**
         * @brief Constructor
         **/
        Reconstructor() {}

        /**
         * @brief Deconstructor
         **/
        ~Reconstructor() {}

        /**
         * @brief Generate polygon meshes using greedy triangulation algorithm
         * 
         * @param cloud_with_normals Cloud of PointNormal
         * @return Resulting polygon meshes of cloud_with_normals
         **/
        pcl::PolygonMesh greedyTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

        /**
         * @brief Generate polygon meshes through Poisson reconstruction method
         * 
         * @param cloud_with_normals Cloud of PointNormal
         * @return Resulting polygon meshes of cloud_with_normals
         **/
        pcl::PolygonMesh poissonReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

        /**
         * @brief Generate polygon meshes utilizing marching cubes method
         * 
         * @param cloud_with_normals Cloud of PointNormal
         * @return Resulting polygon meshes of cloud_with_normals
         **/
        pcl::PolygonMesh marchingCubesReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

        /**
         * @brief Generate polygon meshes based on b-spline surface fitting
         * 
         * @param cloud Cloud of PointXYZ
         * @return Resulting polygon meshes of cloud_with_normals
         **/
        pcl::PolygonMesh bsplineSurfaceFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



    private:
        pcl::GreedyProjectionTriangulation<pcl::PointNormal>   greedy_projection_triangulation;   /**< Object for performing greedy triangulation algorithm */
        pcl::Poisson<pcl::PointNormal>                         poisson;                           /**< Object for performing Poisson reconstruction */
        pcl::PolygonMesh                                       mesh;                              /**< Result of polygon meshes generation */

        /**
         * @brief Converting a cloud of PointXYZ to a set of three-dimensional vectors
         * 
         * @param[in] cloud Cloud of PointXYZ
         * @param[out] data A set of three-dimentional vectors
         **/
        void pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
    };
}
