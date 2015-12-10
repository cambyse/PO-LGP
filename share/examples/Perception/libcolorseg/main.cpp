//#include <System/engine.h>
#include <Gui/graphview.h>
#include <Perception/perception.h>

void TEST(ModuleVision) {
  cout <<registry() <<endl;

  new OpencvCamera;
  addModule<Patcher>(NULL, {"rgb", "patches"});
  new ImageViewer("rgb");
  addModule<GenericDisplayViewer<Patching> >(NULL, {"patches"});
  //S.connect();

  threadOpenModules(true);
  moduleShutdown().waitForValueGreaterThan(0);
  threadCloseModules();

  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  testModuleVision();

  return 0;
}
