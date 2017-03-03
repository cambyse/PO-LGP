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
  double *p=NULL, *c=NULL;
  if(&pts){
    pts.resize(cloud.size(),3);
    p=pts.p;
  }
  if(&cols){
    cols.resize(cloud.size(),3);
    c=cols.p;
  }

  uint i=0;
  for(const PointT& pt:cloud){
    if(p){
      *(p++) = pt.x;
      *(p++) = pt.y;
      *(p++) = pt.z;
    }
    if(c){
      *(c++) = pt.r/255.;
      *(c++) = pt.g/255.;
      *(c++) = pt.b/255.;
    }
    i++;
  }
  if(p) CHECK_EQ(p, pts.p+pts.N, "");
  if(c) CHECK_EQ(c, cols.p+cols.N, "");
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
