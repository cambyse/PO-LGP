#include "test_method.h"

void TestMethod::process(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output) {
    output = input;
}
