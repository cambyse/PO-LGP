#include <MT/vision.h>
#include <NP/uvccamera.h>
#include <NP/camera.h>
#include <MT/opengl.h>

int main(int argc, char** argv){
  //CameraModule camera;
  camera::UVCCamera camera;
  camera.open();
  OpenGL gl;
   
  byteA frame, right_frame;
  double time=MT::realTime();
  uint i;
  for(i=0;i<100;i++){
    //MT::wait(.5);
    camera.step();
    gl.img=&camera.output.rgbL;
    gl.update();
  }
  cout <<"fps=" <<i/(MT::realTime()-time) <<endl;

  write_ppm(camera.output.rgbL,"left.ppm");
  write_ppm(camera.output.rgbL,"right.ppm");

  camera.close();

  return 0;
}
