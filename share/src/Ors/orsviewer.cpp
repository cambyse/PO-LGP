#include "orsviewer.h"

//===========================================================================

void OrsViewer::step(){
  copy.gl().lock.writeLock();
  copy = modelWorld.get();
  copy.gl().lock.unlock();
  copy.gl().update(NULL, false, false, true);
  if(computeCameraView){
    ors::Shape *kinectShape = copy.getShapeByName("endeffKinect");
    if(kinectShape){ //otherwise 'copy' is not up-to-date yet
      copy.gl().lock.writeLock();
      ors::Camera cam = copy.gl().camera;
      copy.gl().camera.setKinect();
      copy.gl().camera.X = kinectShape->X * copy.gl().camera.X;
//      openGlLock();
      copy.gl().renderInBack(true, true, 580, 480);
//      copy.glGetMasks(580, 480, true);
//      openGlUnlock();
      modelCameraView.set() = copy.gl().captureImage;
      modelDepthView.set() = copy.gl().captureDepth;
      copy.gl().camera = cam;
      copy.gl().lock.unlock();
    }
  }
}

//===========================================================================

void OrsPathViewer::open(){
  copy.gl(configurations.name);
}

void OrsPathViewer::step(){
  copy.gl().lock.writeLock();
  configurations.readAccess();
  uint T=configurations().N;
  if(t>=T) t=0;
  if(T) copy.copy(*configurations()(t), true);
  t++;
  configurations.deAccess();
  copy.gl().lock.unlock();
  if(T) copy.gl().update(STRING(" (time " <<t <<'/' <<T <<')').p, false, false, true);
}

//===========================================================================

void ComputeCameraView::open(){
  gl.add(glStandardLight);
  gl.addDrawer(&modelWorld.set()());
}

void ComputeCameraView::step(){
  if(!frame--){
    modelWorld.readAccess();
    gl.renderInBack();
    modelWorld.deAccess();
    cameraView.set() = gl.captureImage;
    frame=skipFrames;
  }
}


