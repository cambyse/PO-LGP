#include "methods.h"

void conv_ArrCloud_PclCloud(pcl::PointCloud<PointT>::Ptr& pcl_cloud,
                         const arr& kinect_points,
                         const arr& kinect_pointColors){
  if(kinect_points.d0<640*480 || kinect_pointColors.d0<640*480) return;
  if(!pcl_cloud)
    pcl_cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(640,480));
  uint i=0;
  for(PointT& p:*pcl_cloud){
    p.x = kinect_points(i,0);
    p.y = kinect_points(i,1);
    p.z = kinect_points(i,2);
    p.r = 255.*kinect_pointColors(i,0);
    p.g = 255.*kinect_pointColors(i,1);
    p.b = 255.*kinect_pointColors(i,2);
    i++;
  }
}
