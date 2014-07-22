#pragma once

#include <Core/array.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;

// THIS IS AN EXAMPLE FOR A PLAIN COMPUTATIONAL ROUTINE
void conv_ArrCloud_PclCloud(pcl::PointCloud<PointT>::Ptr& pcl_cloud,
                            const arr& kinect_points,
                            const arr& kinect_pointColors);

void conv_PclCloud_ArrCloud(arr& kinect_points,
                            arr& kinect_pointColors,
                            const pcl::PointCloud<PointT>::Ptr& pcl_cloud);
