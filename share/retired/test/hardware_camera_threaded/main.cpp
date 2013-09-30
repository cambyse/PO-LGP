#include <perception/perception.h>
#include <hardware/uvccamera.h>
#include <Gui/opengl.h>

int main(int argn,char** argv){
  Image imgL("CameraL"), imgR("CameraR");

  Process *camera = newUVCCamera();

  byteA img;
  OpenGL gl;
  gl.img = &img;
  
  camera->threadLoop();
  double time=MT::realTime();
  int rev=0;
  for(;rev<100;){
    rev=imgL.waitForRevisionGreaterThan(rev);  //sets calling thread to sleep
    img=imgL.get_img(NULL);
    if(!gl.update()) break;
  }
  cout <<"fps=" <<rev/(MT::realTime()-time) <<endl;
  cout <<"trying to close..." <<endl;
  camera->threadClose();
  
  return 0;
}


