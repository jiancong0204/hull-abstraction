#include "moving_least_squares.h"

void moving_least_squares_node::MovingLeastSquares::processing(const sensor_msgs::PointCloud2ConstPtr input_msg) {

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> movingLS;
    pcl::PointCloud<pcl::PointNormal> mlsPoints;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudFilteredWithNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud); 
    tree->setInputCloud(cloud);
    movingLS.setInputCloud(cloud);
    movingLS.setComputeNormals(true);
    movingLS.setPolynomialOrder(3);
    movingLS.setSearchMethod(tree);
    movingLS.setSearchRadius(0.06);
    // Reconstruct
    movingLS.process(mlsPoints);
    pcl::copyPointCloud(mlsPoints, *cloudFiltered);
    pcl::copyPointCloud(mlsPoints, *cloudFilteredWithNormals);
    pcl::toROSMsg(*cloudFiltered, output_msg);
    pub.publish(output_msg);

}

void moving_least_squares_node::MovingLeastSquares::run() {
    sub = nh.subscribe("load_pcd", 1, &MovingLeastSquares::processing,this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("moving_least_squares", 1);
}
