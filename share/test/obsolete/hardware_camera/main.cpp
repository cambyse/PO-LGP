#include <perception/perception.h>
#include <hardware/uvccamera.h>
#include <Gui/opengl.h>

int main(int argc, char** argv){
  Image imgL("CameraL"), imgR("CameraR");

  Process* camera = newUVCCamera();
  
  camera->open();
  OpenGL gl;

  byteA frame, right_frame;
  double time=MT::realTime();
  uint i;
  for(i=0;i<100;i++){
    //MT::wait(.5);
    camera->step();
    gl.img=&imgL.img;
    gl.update();
  }
  cout <<"fps=" <<i/(MT::realTime()-time) <<endl;

  write_ppm(imgL.img,"z.left.ppm");
  write_ppm(imgR.img,"z.right.ppm");

  camera->close();

  return 0;
}
