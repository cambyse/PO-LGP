#include "grab.h"
#include "plane.cpp"
//#include "system.h"

#include <pcl/io/pcd_io.h>

void planeDetectionExample()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  passthroughFilter(cloud,filtered_cloud,1.5,2.5);

  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  normalEstimator(filtered_cloud,normal_cloud,50);

  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
  planeDetector(filtered_cloud,normal_cloud,coefficients_plane,inliers_plane);
}

int main ()
{
  planeDetectionExample();
  return 0;
//  SimpleOpenNIViewer v;
//  v.run ();
//  return 0;
}
