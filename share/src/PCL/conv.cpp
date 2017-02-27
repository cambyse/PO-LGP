#include "conv.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void conv_ArrCloud_PclCloud(Pcl& cloud,
                            const arr& pts, const arr& cols){
  CHECK(pts.nd==3 && pts.d2==3,"");
  cloud.resize(pts.N/3);
  CHECK_EQ(cloud.size(), pts.N/3, "");
  uint i=0;
  for(PointT& p:cloud){
    p.x = pts.elem(i*3+0);
    p.y = pts.elem(i*3+1);
    p.z = pts.elem(i*3+2);
    p.r = 255.*cols.elem(i*3+0);
    p.g = 255.*cols.elem(i*3+1);
    p.b = 255.*cols.elem(i*3+2);
    i++;
  }
}

void conv_PclCloud_ArrCloud(arr& pts,
                            arr& cols,
                            const Pcl& cloud){
  pts.resize(cloud.size(),3);
  cols.resize(cloud.size(),3);
  uint i=0;
  for(const PointT& p:cloud){
    pts(i,0) = p.x;
    pts(i,1) = p.y;
    pts(i,2) = p.z;
    cols(i,0) = p.r/255.;
    cols(i,1) = p.g/255.;
    cols(i,2) = p.b/255.;
    i++;
  }
}

Conv_arr_pcl::Conv_arr_pcl(const char* cloud_name, const char* pts_name, const char* cols_name)
  : Thread(STRING("Conv_arr_pcl"<<cloud_name <<"_" <<pts_name), -1.),
    cloud(this, cloud_name),
    pts(this, pts_name),
    cols(this, cols_name, true){
  threadOpen();
}

Conv_arr_pcl::~Conv_arr_pcl(){
  threadClose();
}

void Conv_arr_pcl::step(){
  conv_ArrCloud_PclCloud( cloud.set(), pts.get(), cols.get() );
}
