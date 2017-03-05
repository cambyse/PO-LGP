#include "conv.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void conv_ArrCloud_PclCloud(Pcl& cloud,
                            const arr& pts, const byteA& rgb){
  CHECK(pts.nd==3 && pts.d2==3,"");
  cloud.resize(pts.N/3);
  CHECK_EQ(cloud.size(), pts.N/3, "");
  double *p=pts.p;
  byte *c=rgb.p;
  for(PointT& pt:cloud){
    pt.x = (*p++); //pts.elem(i*3+0);
    pt.y = (*p++); //pts.elem(i*3+1);
    pt.z = (*p++); //pts.elem(i*3+2);
    pt.r = (*c++); //255.*cols.elem(i*3+0);
    pt.g = (*c++); //255.*cols.elem(i*3+1);
    pt.b = (*c++); //255.*cols.elem(i*3+2);
//    i++;
  }
}

void conv_PclCloud_ArrCloud(arr& pts,
                            byteA& rgb,
                            const Pcl& cloud){
  double *p=NULL;
  byte *c=NULL;
  if(&pts){
    pts.resize(cloud.size(),3);
    p=pts.p;
  }
  if(&rgb){
    rgb.resize(cloud.size(),3);
    c=rgb.p;
  }

  for(const PointT& pt:cloud){
    if(p){
      *(p++) = pt.x;
      *(p++) = pt.y;
      *(p++) = pt.z;
    }
    if(c){
      *(c++) = pt.r;
      *(c++) = pt.g;
      *(c++) = pt.b;
    }
  }
  if(p) CHECK_EQ(p, pts.p+pts.N, "");
  if(c) CHECK_EQ(c, rgb.p+rgb.N, "");
}


Conv_arr_pcl::Conv_arr_pcl(const char* cloud_name, const char* pts_name, const char* rgb_name)
  : Thread(STRING("Conv_arr_pcl"<<cloud_name <<"_" <<pts_name), -1.),
    cloud(this, cloud_name),
    pts(this, pts_name),
    rgb(this, rgb_name, true){
  threadOpen();
}

Conv_arr_pcl::~Conv_arr_pcl(){
  threadClose();
}

void Conv_arr_pcl::step(){
  copyPts = pts.get();
  copyRgb = rgb.get();
  if(!copyPts.N || copyRgb.N!=copyRgb.N) return;
  conv_ArrCloud_PclCloud( cloud.set(), copyPts, copyRgb );
}


