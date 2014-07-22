#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Core/geo.h>
#include <Gui/opengl.h>
#include <Ors/ors.h>

#include "methods.h"

typedef pcl::PointXYZRGB PointT;

void glDrawPrimitives(void* classP);

struct Primitive{
  virtual void glDraw() = 0;
  virtual ~Primitive(){}
};

struct Plane:Primitive{
  float nx,ny,nz,c;
  Plane(float _nx, float _ny, float _nz, float _c):nx(_nx),ny(_ny),nz(_nz),c(_c){}
  ~Plane(){}
  void glDraw();
};

struct CloudView:Primitive{
  pcl::PointCloud<PointT>::Ptr cloud;
  arr pts,cols;
  CloudView(const pcl::PointCloud<PointT>::Ptr& _cloud):cloud(_cloud){}
  void glDraw();
};

struct DisplayPrimitives{
  MT::Array<Primitive*> P;
  ors::KinematicWorld G;

  void glDraw(){
    for(Primitive* p:P) p->glDraw();
    G.glDraw();
  }
};
