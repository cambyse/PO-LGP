#include <MT/vision.h>
#include <NP/camera.h>
#include <NP/uvccamera.h>
#include <MT/opengl.h>

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  //CameraModule camera;
  camera::UVCCamera camera;

  byteA img;
  OpenGL gl;
  gl.img = &img;
  
  camera.threadOpen();
  camera.step();
  camera.threadLoop();
  for(;;){
    camera.output.readAccess(NULL);
    img=camera.output.rgbL;
    camera.output.deAccess(NULL);
    //if(cvShow(img,"1")==27) break;
    if(gl.update()==27) break;
  }
  cout <<"trying to close..." <<endl;
  camera.threadStop();
  camera.threadClose();
  
  return 0;
}


