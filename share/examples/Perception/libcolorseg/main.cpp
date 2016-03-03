//#include <System/engine.h>
#include <Gui/graphview.h>
#include <Perception/perception.h>

void TEST(ModuleVision) {
  OpencvCamera cam;
  Patcher pather("rgb", "patches");
  
  ImageViewer v1("rgb");
  GenericDisplayViewer<Patching> v2("patches");

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
