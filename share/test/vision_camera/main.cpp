#include <perception/perception.h>
#include <hardware/uvccamera.h>
#include <MT/opengl.h>

int main(int argc, char** argv){
  //CameraModule camera;
  UVCCamera camera;
  Image imgL("cameraL"), imgR("cameraR");
  
  camera.open();
  OpenGL gl;
   
  byteA frame, right_frame;
  double time=MT::realTime();
  uint i;
  for(i=0;i<100;i++){
    //MT::wait(.5);
    camera.step();
    gl.img=&imgL.img;
    gl.update();
  }
  cout <<"fps=" <<i/(MT::realTime()-time) <<endl;

  write_ppm(imgL.img,"left.ppm");
  write_ppm(imgR.img,"right.ppm");

  camera.close();

  return 0;
}
