#include <Perception/perception.h>
#include <Gui/opengl.h>
//#include <System/engine.h>

float angle;

void draw(void*){
  glStandardLight(NULL);
  glColor(1, 0, 0);
  glRotatef(angle, 0, 1, 0);
  glutSolidTeapot(1.);
}

struct EncodingExampleSystem{
  ACCESS(byteA, rgb);
  EncodingExampleSystem(){
#ifdef MLR_OPENCV
    addModule<OpencvCamera>(NULL, Module::loopFull);
    new ImageViewer("rgb");
    addModule<VideoEncoder>(NULL, {"rgb"} /*,Module::listenFirst*/ );
#else
    NICO
#endif
    //connect();
  }
};

void threadedRun(){
  EncodingExampleSystem S;
  //cout <<S <<endl;
  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();
  cout << "bye bye" << endl;
}

int main(int argc,char **argv){
  threadedRun();

  return 0;
}

