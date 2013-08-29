#include <System/engine.h>
//#include <Gui/graphview.h>

//NOTE: no actual perception code is included - only system!
void lib_Perception(); //this is enough to ensure the linking and loading of registry entries

int main(int argn,char **argv) {
  MT::initCmdLine(argn,argv);
  lib_Perception();
  cout <<registry() <<endl;

  System S;
  S.addModule("OpencvCamera", NULL, ModuleThread::loopFull);
  S.addModule("CvtGray");
  S.addModule("MotionFilter");
  S.addModule("ImageViewer", NULL, STRINGS("motion"), ModuleThread::listenFirst, .0);
  S.connect();

  cout <<S <<endl;

  engine().enableAccessLog();
//  engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  engine().open(S);

//  KeyValueGraph g = S.graph();
//  GraphView gv(g); gv.watch();

  if(engine().mode==Engine::serial){
    for(uint i=0;i<100;i++){ engine().step(S); }
  }else{
    MT::wait(10.);
  }

  engine().close(S);

  return 0;
}
