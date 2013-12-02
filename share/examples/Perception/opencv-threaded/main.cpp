#include <System/engine.h>
#include <Gui/graphview.h>

//NOTE: no actual perception code is included - only system!
void lib_Perception(); //this is enough to ensure the linking and loading of registry entries

void TEST(ModuleVision) {
  lib_Perception();
  cout <<registry() <<endl;

  System S;
  S.addModule("OpencvCamera", NULL, ModuleThread::loopFull);
  S.addModule("CvtGray");
  S.addModule("MotionFilter");
  S.addModule("ImageViewer", NULL, STRINGS("motion"), ModuleThread::listenFirst);
  S.addModule("VideoEncoder", NULL, STRINGS("rgb"), ModuleThread::listenFirst);
  S.addModule("VideoEncoder", "MyMotionWriter", STRINGS("motion"), ModuleThread::listenFirst);
  S.connect();

  cout <<S <<endl;

  engine().enableAccessLog();
  //engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  engine().open(S);

  KeyValueGraph g = S.graph();
  GraphView gv(g); gv.update();

  if(engine().mode==Engine::serial){
    for(uint i=0;i<100;i++){ engine().step(S); }
  }else{
    MT::wait(6.);
  }

  engine().close(S);
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  testModuleVision();

  return 0;
}
