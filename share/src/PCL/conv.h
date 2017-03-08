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

void conv_ArrCloud_PclCloud(Pcl& cloud, const arr& pts, const byteA& rgb);
void conv_PclCloud_ArrCloud(arr& pts, byteA& rgb, const Pcl& cloud);
inline arr PclPoints(const Pcl& cloud){ arr pts; conv_PclCloud_ArrCloud(pts, NoByteA, cloud); return pts; }

struct Conv_arr_pcl : Thread{
  Access_typed<Pcl> cloud;
  Access_typed<arr> pts;
  Access_typed<byteA> rgb;
  byteA copyRgb;
  arr copyPts;
  bool kinectColorBGRSwap;
  Conv_arr_pcl(const char* cloud_name, const char* pts_name, const char* rgb_name);
  ~Conv_arr_pcl();
  void open(){}
  void step();
  void close(){}
};

