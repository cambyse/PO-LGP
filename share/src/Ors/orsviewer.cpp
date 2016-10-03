/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "orsviewer.h"

#include <iomanip>

//===========================================================================

void OrsViewer::open(){
  copy.gl(modelWorld.name);
}

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
  configurations.deAccess();
  copy.gl().lock.unlock();
  if(T){
    copy.gl().captureImg=writeToFiles;
    copy.gl().update(STRING(" (time " <<tprefix+int(t) <<'/' <<tprefix+int(T) <<')').p, false, false, true);
    if(writeToFiles) write_ppm(copy.gl().captureImage,STRING("vid/z.path."<<std::setw(3)<<std::setfill('0')<<tprefix+int(t)<<".ppm"));
  }
  t++;
}

//===========================================================================

void changeColor(void*){  orsDrawColors=false; glColor(.5, 1., .5, .7); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

void OrsPoseViewer::open() {
  gl.add(glStandardScene, 0);
  gl.camera.setDefault();

  gl.add(changeColor2);
  for(uint i=0;i<copies.N;i++){
    gl.add(*copies(i));
    gl.add(changeColor);
  }
  gl.add(changeColor2);
}

void OrsPoseViewer::step(){
  gl.lock.writeLock();
  for(uint i=0;i<copies.N;i++){
    arr q=poses(i)->get();
    if(q.N==copies(i)->getJointStateDimension())
      copies(i)->setJointState(q);
  }
  gl.lock.unlock();
  gl.update("PoseViewer", false, false, true);
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


