#include <pcl/common/common_headers.h>

void box_smoothing(int width,
                   const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud);

void box_smoothing(int width, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
