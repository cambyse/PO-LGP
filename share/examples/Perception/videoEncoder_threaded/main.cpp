#include <Perception/perception.h>
#include <Gui/opengl.h>
#include <System/engine.h>

float angle;

void draw(void*){
  glStandardLight(NULL);
  glColor(1, 0, 0);
  glRotatef(angle, 0, 1, 0);
  glutSolidTeapot(1.);
}

struct EncodingExampleSystem:System{
  ACCESS(byteA, rgb);
  EncodingExampleSystem(){
#ifdef MT_OPENCV
    addModule<OpencvCamera>(NULL, Module_Thread::loopFull);
    addModule<ImageViewer>(NULL, STRINGS("rgb"), Module_Thread::listenFirst);
    addModule<VideoEncoder>(NULL, STRINGS("rgb"), Module_Thread::listenFirst);
#else
    NICO
#endif
    connect();
  }
};

void threadedRun(){
  EncodingExampleSystem S;
  cout <<S <<endl;
  engine().open(S);

  engine().shutdown.waitForSignal();

  engine().close(S);
  cout << "bye bye" << endl;
}

int main(int argc,char **argv){
  threadedRun();

  return 0;
}

