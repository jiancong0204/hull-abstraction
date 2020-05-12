#include "hull_abstraction/preprocessor.h"

void hull_abstraction::Preprocessor::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    // Down sampling to reduce the number of points without losing much precision
    approximate_voxel_grid.setInputCloud(cloud);
    approximate_voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);  //Volume of voxel(AABB) is 1 cm3
    approximate_voxel_grid.setDownsampleAllData(false); //No down sampling based on RGB information
    approximate_voxel_grid.filter(*filtered_cloud);
}

void hull_abstraction::Preprocessor::statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    statistical_outlier_removal.setInputCloud(cloud);
    statistical_outlier_removal.setMeanK(50);
    statistical_outlier_removal.setStddevMulThresh(1.0);
    statistical_outlier_removal.filter(*filtered_cloud);
}

void hull_abstraction::Preprocessor::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    pass_through.setInputCloud(cloud);
    pass_through.setFilterFieldName("z"); //opration for z-axis
    pass_through.setFilterLimits(0.0, 400.0); //range
    //passThrough.setFilterLimitsNegative(true);
    pass_through.filter(*filtered_cloud);
}

void hull_abstraction::Preprocessor::conditionalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
    conditional_removal.setCondition(range_condition);
    conditional_removal.setInputCloud(cloud);
    conditional_removal.setKeepOrganized(true);
    conditional_removal.filter(*filtered_cloud);
}

void hull_abstraction::Preprocessor::radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    radius_outlier_removal.setInputCloud(cloud);
    radius_outlier_removal.setRadiusSearch(100);
    radius_outlier_removal.setMinNeighborsInRadius(2); //delete the point whose number of neighbours is less than 2
    radius_outlier_removal.filter(*filtered_cloud);
}

void hull_abstraction::Preprocessor::appendNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setKSearch(20);
    normal_estimation.compute(*normals);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

void hull_abstraction::Preprocessor::movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr smoothed_cloud_with_normals)
{
    double resolution = pcl_utilization::computeCloudResolution(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    moving_least_squares.setInputCloud(cloud);
    moving_least_squares.setComputeNormals(true); // MLS offers a method to estimate normals
    moving_least_squares.setPolynomialOrder(3); // Polynomial order used to fit the curve. The default value is 2, however 3 or 4 is better in my case
    moving_least_squares.setSearchMethod(tree);
    moving_least_squares.setSearchRadius(10 * resolution);
    // Reconstruct
    moving_least_squares.process(mls_points);
    pcl::copyPointCloud(mls_points, *smoothed_cloud);
    pcl::copyPointCloud(mls_points, *smoothed_cloud_with_normals);
}
