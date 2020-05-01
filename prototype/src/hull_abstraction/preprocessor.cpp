#include "include/hull_abstraction/preprocessor.h"

void hull_abstraction::Preprocessor::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered)
{
    // Down sampling to reduce the number of points without losing much precision
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);  //Volume of voxel(AABB) is 1 cm3
    voxelGrid.setDownsampleAllData(false); //No down sampling based on RGB information
    voxelGrid.filter(*cloudFiltered);
}

void hull_abstraction::Preprocessor::statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered)
{
    statistical.setInputCloud(cloud);
    statistical.setMeanK(50);
    statistical.setStddevMulThresh(1.0);
    statistical.filter(*cloudFiltered);
}

void hull_abstraction::Preprocessor::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered)
{
    passThrough.setInputCloud(cloud);
    passThrough.setFilterFieldName("z"); //opration for z-axis
    passThrough.setFilterLimits(0.0, 400.0); //range
    //passThrough.setFilterLimitsNegative(true);
    passThrough.filter(*cloudFiltered);
}

void hull_abstraction::Preprocessor::conditionalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered)
{
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr rangeCondition(new pcl::ConditionAnd<pcl::PointXYZ>());
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
    condition.setCondition(rangeCondition);
    condition.setInputCloud(cloud);
    condition.setKeepOrganized(true);
    condition.filter(*cloudFiltered);
}

void hull_abstraction::Preprocessor::radiusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered)
{
    radius.setInputCloud(cloud);
    radius.setRadiusSearch(100);
    radius.setMinNeighborsInRadius(2); //delete the point whose number of neighbours is less than 2
    radius.filter(*cloudFiltered);
}

void hull_abstraction::Preprocessor::appendNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setKSearch(20);
    normalEstimation.compute(*normals);
    pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
}

void hull_abstraction::Preprocessor::movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSmoothed, pcl::PointCloud<pcl::PointNormal>::Ptr cloudSmoothedWithNormals)
{
    double resolution = hull_abstraction::computeCloudResolution(cloud);
    std::cout << resolution << std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    movingLS.setInputCloud(cloud);
    movingLS.setComputeNormals(true);
    movingLS.setPolynomialOrder(3);
    movingLS.setSearchMethod(tree);
    movingLS.setSearchRadius(10 * resolution);
    // Reconstruct
    movingLS.process(mlsPoints);
    pcl::copyPointCloud(mlsPoints, *cloudSmoothed);
    pcl::copyPointCloud(mlsPoints, *cloudSmoothedWithNormals);
}
