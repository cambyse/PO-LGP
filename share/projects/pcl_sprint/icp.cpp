#include "icp.h"

void ICP::apply(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out) {
    // create icp object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // create xyz clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    // copy over input and target
    pcl::copyPointCloud(*cloud_in, *cloud_in_xyz);
    pcl::copyPointCloud(*cloud_target, *cloud_target_xyz);
    // set input and target
    icp.setInputCloud(cloud_in_xyz);
    icp.setInputTarget(cloud_target_xyz);
    // perform alignment
    icp.align(*cloud_out_xyz);
    // copy over result
    pcl::copyPointCloud(*cloud_out_xyz, *cloud_out);
    // print some info
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
}
