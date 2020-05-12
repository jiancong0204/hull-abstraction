/**
  * @file
  * @brief This file contains the declaration of the Preprocessor class
  *
  * @author Jiancong Zheng
  **/

#pragma once

#include "pcl_utilization/pcl_utilization.h"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/approximate_voxel_grid.h> // Head file for approximate voxel grid method
#include <pcl/filters/statistical_outlier_removal.h> // Head file for statistical outlier removal
#include <pcl/filters/passthrough.h> // Head file for pass-through filter
#include <pcl/filters/conditional_removal.h> // Head file for conditional filter
#include <pcl/filters/radius_outlier_removal.h> // Head file for radius outlier removal method
#include <pcl/surface/mls.h> // Head file for moving least squares method

namespace hull_abstraction
{
    /**
     * @brief The Preprocessor class
     *
     * This class wraps some preprocessing methods for point cloud data
     **/
    class Preprocessor
    {
    public:
        /**
         * @brief Constructor
         **/
        Preprocessor() {}

        /**
         * @brief Deconstructor
         **/
        ~Preprocessor() {}

        /**
         * @brief Implementation of the voxel grid filter to perform down sampling
         * @param cloud Cloud of PointXYZ
         * @param filtered_cloud Cloud of PointXYZ
         **/
        void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /**
         * @brief Removing outliers through the statitical filter
         * @param cloud Cloud of PointXYZ
         * @param filtered_cloud Cloud of PointXYZ
         **/
        void statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /**
         * @brief Implementation of the pass-through filter
         * @param cloud Cloud of PointXYZ
         * @param filtered_cloud Cloud of PointXYZ
         **/
        void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /**
         * @brief Implementation of the conditional filter
         * @param cloud Cloud of PointXYZ
         * @param filtered_cloud Cloud of PointXYZ
         **/
        void conditionalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /**
         * @brief Implementation of the radius filter
         * @param cloud Cloud of PointXYZ
         * @param filtered_cloud Cloud of PointXYZ
         **/
        void radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        /**
         * @brief Estimating normals and append those to the input cloud
         * @param cloud Cloud of PointXYZ
         * @param cloud_with_normals Cloud of PointNormal
         **/
        void appendNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

        /**
         * @brief Performing MLS (Moving Least Squares) algorithm for data smoothing and normal estimation
         * @param cloud Cloud of PointXYZ
         * @param smoothed_cloud Cloud of PointXYZ
         **/
        void movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud, 
            pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_cloud_with_normals);

    private:
        pcl::ApproximateVoxelGrid<pcl::PointXYZ>                   approximate_voxel_grid;        /**< Object for approximate voxel grid */
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ>              statistical_outlier_removal;   /**< Object for statistical outlier removal */
        pcl::PassThrough<pcl::PointXYZ>                            pass_through;                  /**< Object for pass-through filter */
        pcl::ConditionalRemoval<pcl::PointXYZ>                     conditional_removal;           /**< Object for conditional removal */
        pcl::RadiusOutlierRemoval<pcl::PointXYZ>                   radius_outlier_removal;        /**< Object for radius outlier removal */
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>          normal_estimation;             /**< Object for normal estimation */
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>   moving_least_squares;          /**< Object for moving least squares method*/
        pcl::PointCloud<pcl::PointNormal>                          mls_points;                    /**< Point cloud used to stored the result of MLS*/
    };
}
