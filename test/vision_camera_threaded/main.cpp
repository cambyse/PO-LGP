#define MT_IMPLEMENTATION

#include <MT/vision.h>
#include <NP/camera.h>
#include <NP/uvccamera.h>
#include <MT/opengl.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

#if 0
  camera::UVCCamera camera;
#else
  BumblebeeModule camera;
#endif

  byteA img;
  OpenGL gl;
  gl.img = &img;
  
  camera.threadOpen();
  camera.step();
  camera.threadLoop();
  for(;;){
    camera.lock.readLock();
    img=camera.rgbL;
    camera.lock.unlock();
    //if(cvShow(img,"1")==27) break;
    if(gl.update()==27) break;
  }
  cout <<"trying to close..." <<endl;
  camera.threadLoopStop();
  camera.threadClose();
  
  return 0;
}


