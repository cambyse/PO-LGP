#define MT_IMPLEMENTATION

#include <MT/vision.h>
#include <NP/uvccamera.h>
#include <NP/uvccamera.cpp>
#include <NP/camera.h>
#include <NP/cameracalibrator.h>

void testCam(){
#if 0
   camera::UVCCamera camera;
#else
   BumblebeeModule camera;
#endif
   camera.open();
   
   byteA frame, right_frame;
   double time=MT::realTime();
   uint i;
   for(i=0;i<100;i++){
     //MT::wait(.5);
     camera.step();
     if(cvShow(camera.rgbL)==27) break;
   }
   cout <<"fps=" <<i/(MT::realTime()-time) <<endl;
   
   camera.close();
}

void calib(){
  BumblebeeModule cam;
  cam.open();

  np::CameraCalibrator cc(cam.camera,8,6);
  cc.run();
}


int main(int argc, char** argv){

//   testCam();
  calib();
  return 0;
}
