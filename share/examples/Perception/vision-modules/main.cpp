#include <System/engine.h>
#include <Gui/graphview.h>
#include <Perception/perception.h>

//NOTE: no actual perception code is included - only system!
void lib_Perception(); //this is enough to ensure the linking and loading of registry entries

void TEST(ModuleVision) {
  lib_Perception();
  cout <<registry() <<endl;

  System S;
  S.addModule("OpencvCamera", NULL, Module_Thread::loopFull);
  S.addModule("CvtGray");
  S.addModule("MotionFilter");
  S.addModule("DifferenceFilter", NULL, {"rgb", "ground", "diffImage"});
  S.addModule("CannyFilter", NULL, {"gray", "canny"});
  S.addModule("Patcher", NULL, {"rgb", "patches"});
  //  S.addModule("SURFer", NULL, {"gray", "features"});

  S.addModule("ImageViewer", NULL, {"rgb"});
  S.addModule("ImageViewer", NULL, {"gray"});
  S.addModule("ImageViewer", NULL, {"motion"});
  S.addModule("ImageViewer", NULL, {"diffImage"});
  S.addModule("ImageViewer", NULL, {"canny"});
  S.addModule<GenericDisplayViewer<Patching> >(NULL, {"patches"});
//  S.addModule<GenericDisplayViewer<SURFfeatures> >(NULL, {"features"});

  //  S.addModule("VideoEncoder", NULL, {"rgb"}, Module_Thread::listenFirst);
  //  S.addModule("VideoEncoder", "MyMotionWriter", {"motion"}, Module_Thread::listenFirst);

  S.connect();

  cout <<S <<endl;

  engine().enableAccessLog();
//  engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  engine().open(S);

  KeyValueGraph g = S.graph();
  GraphView gv(g); gv.update();

  if(engine().mode==Engine::serial){
    for(uint i=0;i<100;i++){ engine().step(S); }
  }else{
    MT::wait(60.);
  }

  engine().close(S);

  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  testModuleVision();

  return 0;
}
