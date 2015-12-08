#include <Hardware/kinect/kinect.h>
#include <Gui/opengl.h>

void testKinect(){

  OpenGL gl("FREENECT",640,480);

  KinectThread kin;
  kin.verbose=1;
  kin.threadLoop();

  for(uint t=0;t<50;t++){
    kin.kinect_rgb.waitForNextRevision();
    gl.background = kin.kinect_rgb.get();
    gl.update();
    kin.glViewKeys('x'); //moving down always...
  }

  kin.threadClose();
}

int main(int argc, char* argv[]){
  testKinect();
  return 0;
}

