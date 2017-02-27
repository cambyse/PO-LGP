#include "pclViewer.h"
#include "conv.h"
#include <Geo/mesh.h>
#include <Gui/opengl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct sPclViewer{
  OpenGL gl;
  sPclViewer(const char* tit):gl(tit,640,480){}
  mlr::Mesh pc;
};

void glDrawAxes(void*){
  glDrawAxes(1.);
}

PclViewer::PclViewer(const char* cloud_name)
  : Thread(STRING("PclViewer_"<<cloud_name)),
    cloud(this, cloud_name, true){
  threadOpen();
}

PclViewer::~PclViewer(){
  threadClose();
}

void PclViewer::open(){
  s = new sPclViewer(STRING("PclViewer: "<<cloud.name));
  s->gl.add(glStandardScene);
  s->gl.add(s->pc);
  s->gl.camera.setDefault();
}

void PclViewer::close(){
  delete s;
}

void PclViewer::step(){
  cloud.readAccess();
  if(cloud().size()){
    conv_PclCloud_ArrCloud(s->pc.V, s->pc.C, cloud());
  }
  cloud.deAccess();
  s->gl.update();
}

