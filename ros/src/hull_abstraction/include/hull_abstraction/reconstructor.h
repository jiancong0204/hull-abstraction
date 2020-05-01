#pragma once
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include "hull_abstraction/preprocessor.h"

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

/* Class that wraps the generation of polygon meshes. */

namespace hull_abstraction {
    class Reconstructor {
    public:

        /* Constructor & Deconstructor*/
        Reconstructor() {}
        ~Reconstructor() {}

        /* Function that generates polygon meshes using greedy triangulation algorithm. 
           Returns a PolygonMesh. */
        pcl::PolygonMesh greedyTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);

        /* Function that generates polygon meshes using greedy triangulation algorithm. */
        pcl::PolygonMesh poissonReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);

        /* Function that generates polygon meshes using marching cubes algorithm. */
        pcl::PolygonMesh marchingCubesReconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals);

        /* Function that implements b-spline surface fitting.
           Input: cloud of PointXYZ whithout normal estimation.
           Return pcl::PolygonMesh. */
        pcl::PolygonMesh bsplineSurfaceFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



    private:
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedyProjection;
        pcl::Poisson<pcl::PointNormal> poisson;
        pcl::PolygonMesh mesh;
        /* Function that converts pcl::PointCloud<pcl::PointXYZ>::Ptr cloud to pcl::on_nurbs::vector_vec3d.
           Result is stored in 'data'.*/
        void pointCloud2Vector3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
    };
}
