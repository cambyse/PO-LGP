//#include <System/engine.h>
#include <Gui/graphview.h>
#include <Perception/perception.h>

void TEST(ModuleVision) {
  cout <<registry() <<endl;

  System S;
  S.addModule<OpencvCamera>(NULL, Module::loopFull);
  S.addModule<Patcher>(NULL, {"rgb", "patches"});
  new ImageViewer("rgb");
  S.addModule<GenericDisplayViewer<Patching> >(NULL, {"patches"});
  //S.connect();

  cout <<S <<endl;

  threadOpenModules(true);
  shutdown().waitForValueGreaterThan(0);
  threadCloseModules();

  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  testModuleVision();

  return 0;
}
