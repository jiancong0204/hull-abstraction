#pragma once
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include "hull_abstraction/functions.h"

/* Class that wraps the preprocessing of a point cloud. */

namespace hull_abstraction {
    class Preprocessor
    {
    public:
        /* Constructor */
        Preprocessor() {}
        ~Preprocessor() {}

        /* Function that implements a voxel grid filter to perform down sampling. 
           Input: a cloud of PointXYZ
           Output: a set of PointXYZ
           The output is stored in cloudFiltered. */
        void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);

        /* Function that implements a statistical filter to remove outliers. 
           The output is stored in cloudFiltered.*/
        void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);

        /* Function that implements a pass-through filter. 
           The output is stored in cloudFiltered.*/
        void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);

        /* Function that implements a conditional filter. 
           The output is stored in cloudFiltered.*/
        void conditionalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);

        /* Function that implements a radius filter. 
           The output is stored in cloudFiltered.*/
        void radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);

        /* Function that estimates the normals and appends them to the cloud. 
           The output is stored in cloudFiltered.*/
        void appendNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloudFiltered);

        /* Function that implements MLS (Moving Least Squares) algorithm for data smoothing (up sampling) and improved normal estimation. 
           The first output is a point cloud after applying MLS, which is stored in cloudSmoothed.
           The second output is the cloud with normals, which is stored in cloudSmoothedWithNormals*/
        void movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSmoothed, 
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudSmoothedWithNormals);

    private:
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelGrid;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical;
        pcl::PassThrough<pcl::PointXYZ> passThrough;
        pcl::ConditionalRemoval<pcl::PointXYZ> condition;
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius;
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> movingLS;
        pcl::PointCloud<pcl::PointNormal> mlsPoints;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    };
}
