#include <pcl/common/common_headers.h>

void depth_filter(double depth,
                  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input_cloud,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output_cloud);

void depth_filter(double depth,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
