#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <boost/shared_ptr.hpp>

namespace pcl{
  struct PointXYZRGB;
  template<class T> struct PointCloud;
}
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Pcl;

void conv_ArrCloud_PclCloud(Pcl& cloud, const arr& pts, const arr& cols);
void conv_PclCloud_ArrCloud(arr& pts, arr& cols, const Pcl& cloud);

struct Conv_arr_pcl : Thread{
  Access_typed<Pcl> cloud;
  Access_typed<arr> pts;
  Access_typed<arr> cols;
  Conv_arr_pcl(const char* cloud_name, const char* pts_name, const char* cols_name);
  ~Conv_arr_pcl();
  void open(){}
  void step();
  void close(){}
};

