#define MT_IMPLEMENTATION

#include <MT/vision.h>
#include <NP/uvccamera.h>
#include <NP/camera.h>
#include <MT/opengl.h>

int main(int argc, char** argv){
#if 1
   camera::UVCCamera camera;
#else
   BumblebeeModule camera;
#endif
   camera.open();
   OpenGL gl;
   
   byteA frame, right_frame;
   double time=MT::realTime();
   uint i;
   for(i=0;i<100;i++){
     //MT::wait(.5);
     camera.step();
     gl.img=&camera.rgbL;
     gl.update();
   }
   cout <<"fps=" <<i/(MT::realTime()-time) <<endl;

   write_ppm(camera.rgbL,"z.ppm");

   camera.close();

   return 0;
}
