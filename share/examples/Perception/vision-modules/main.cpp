#include <Gui/graphview.h>
#include <Perception/perception.h>

#if 0
void TEST(ModuleVision) {
  cout <<registry() <<endl;

  System S;
  S.addModule("OpencvCamera", NULL, Module::loopFull);
  S.addModule("CvtGray");
  S.addModule("MotionFilter");
  S.addModule("DifferenceFilter", NULL, {"rgb", "ground", "diffImage"});
  S.addModule("CannyFilter", NULL, {"gray", "canny"});
//  S.addModule("Patcher", NULL, {"rgb", "patches"});
//  //  S.addModule("SURFer", NULL, {"gray", "features"});

  S.addModule("ImageViewer", NULL, {"rgb"});
  S.addModule("ImageViewer", NULL, {"gray"});
  S.addModule("ImageViewer", NULL, {"motion"});
  S.addModule("ImageViewer", NULL, {"diffImage"});
  S.addModule("ImageViewer", NULL, {"canny"});
//  S.addModule<GenericDisplayViewer<Patching> >(NULL, {"patches"});
//  S.addModule<GenericDisplayViewer<SURFfeatures> >(NULL, {"features"});

  //  S.addModule("VideoEncoder", NULL, {"rgb"}, Module::listenFirst);
  //  S.addModule("VideoEncoder", "MyMotionWriter", {"motion"}, Module::listenFirst);

  S.connect();

  cout <<S <<endl;

  engine().enableAccessLog();
//  engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  engine().open(S);

  Graph g = S.graph();
//  GraphView gv(g); gv.update();

  if(engine().mode==Engine::serial){
    for(uint i=0;i<100;i++){ engine().step(S); }
  }else{
    mlr::wait(60.);
  }

  engine().close(S);

  cout <<"bye bye" <<endl;
}
#endif

void TEST(ModuleVision2) {
  cout <<registry() <<endl;

  OpencvCamera cam;
  CvtGray cvtGray;
  MotionFilter mofi;
  DifferenceFilter difi("rgb", "ground", "diffImage");
  CannyFilter canniFi("gray", "canny");

  ImageViewer iv1("rgb");
  ImageViewer iv2("gray");
  ImageViewer iv3("motion");
  ImageViewer iv4("diffImage");
  ImageViewer iv5("canny");

  cout <<registry() <<endl;

  threadOpenModules(true);
  for(uint i=0;i<30;i++){
    mlr::wait(1.);
    modulesReportCycleTimes();
  }
  threadCloseModules();

  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  testModuleVision2();

  return 0;
}
