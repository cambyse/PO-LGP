#include "viewer.h"
#include <Gui/opengl.h>
#include <Geo/mesh.h>

//===========================================================================
//
// ImageViewer
//

struct sImageViewer{
  OpenGL gl;
  sImageViewer(const char* tit) : gl(tit) {}
};

ImageViewer::ImageViewer(const char* img_name) : Thread(STRING("ImageViewer_"<<img_name), -1), img(this, img_name, true){
  threadOpen();
}

ImageViewer::~ImageViewer(){ threadClose(); }

void ImageViewer::open(){
  s = new sImageViewer(STRING("ImageViewer: "<<img.data->name));
  s->gl.openWindow();
  s->gl.update();
}

void ImageViewer::close(){ delete s; }

void ImageViewer::step(){
  s->gl.dataLock.writeLock();
  s->gl.background = img.get();
  if(flipImage) flip_image(s->gl.background);
  s->gl.dataLock.unlock();
  if(!s->gl.background.N) return;
  if(s->gl.height!= s->gl.background.d0 || s->gl.width!= s->gl.background.d1)
    s->gl.resize(s->gl.background.d1, s->gl.background.d0);
  s->gl.update(name, false, false, true);
}


//===========================================================================
//
// PointCloudViewer
//

struct sPointCloudViewer{
  OpenGL gl;
  sPointCloudViewer():gl("PointCloudViewer",640,480){}
  mlr::Mesh pc;
};

void glDrawAxes(void*){
  glDrawAxes(1.);
}

PointCloudViewer::PointCloudViewer(const char* pts_name, const char* cols_name)
  : Thread(STRING("PointCloudViewer_"<<pts_name <<'_' <<cols_name), .1),
    pts(this, pts_name, true),
    cols(this, cols_name){
  threadLoop();
}

PointCloudViewer::~PointCloudViewer(){
  threadClose();
}

void PointCloudViewer::open(){
  s = new sPointCloudViewer;
#if 0
  s->gl.add(glDrawAxes);
  s->gl.add(s->pc);
  s->gl.camera.setKinect();
  //  s->gl.reportSelects = true;
#else
  s->gl.add(glStandardScene);
  s->gl.add(s->pc);
  s->gl.camera.setDefault();
#endif
}

void PointCloudViewer::close(){
  delete s;
}

void PointCloudViewer::step(){
  s->pc.V=pts.get();
  s->pc.C=cols.get();
  s->gl.update();
}

