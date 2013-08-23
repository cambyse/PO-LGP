#include <Perception/perception.h>
//#include <views/views.h>
#include <System/engine.h>
#include <Gui/graphview.h>

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);
  loadPerception();
  cout <<registry() <<endl;

  System S;
  S.addModule("OpencvCamera",NULL,ModuleThread::loopFull,0);
  S.addModule("CvtGray");
  S.addModule("MotionFilter");
  S.addModule("ImageViewer",NULL,STRINGS("motion"), ModuleThread::listenFirst,.0);
  S.complete();

  cout <<S <<endl;

  engine().enableAccessLog();
  //engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  engine().open(S);

//  KeyValueGraph g = S.graph();
//  GraphView gv(g); gv.watch();

  MT::wait(10.);

  engine().close(S);

  return 0;
}
