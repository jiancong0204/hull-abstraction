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
namespace hull_abstraction
{
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
        void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /* Function that implements a statistical filter to remove outliers. 
        The output is stored in cloudFiltered.*/
        void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /* Function that implements a pass-through filter. 
        The output is stored in cloudFiltered.*/
        void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /* Function that implements a conditional filter. 
        The output is stored in cloudFiltered.*/
        void conditionalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /* Function that implements a radius filter. 
        The output is stored in cloudFiltered.*/
        void radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /* Function that estimates the normals and appends them to the cloud. 
        The output is stored in cloudFiltered.*/
        void appendNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

        /* Function that implements MLS (Moving Least Squares) algorithm for data smoothing (up sampling) and improved normal estimation. 
        The first output is a point cloud after applying MLS, which is stored in cloudSmoothed.
        The second output is the cloud with normals, which is stored in cloudSmoothedWithNormals*/
        void movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud, 
            pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_cloud_with_normals);

    private:
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_grid;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;
        pcl::PassThrough<pcl::PointXYZ> pass_through;
        pcl::ConditionalRemoval<pcl::PointXYZ> conditional_removal;
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_removal;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> moving_least_squares;
        pcl::PointCloud<pcl::PointNormal> mls_points;
    };
}
