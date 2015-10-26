#include <System/engine.h>
#include <Gui/graphview.h>
#include <Perception/perception.h>

void TEST(ModuleVision) {
  cout <<registry() <<endl;

  System S;
  S.addModule<OpencvCamera>(NULL, Module::loopFull);
  S.addModule<Patcher>(NULL, {"rgb", "patches"});
  S.addModule<ImageViewer>(NULL, {"rgb"});
  S.addModule<GenericDisplayViewer<Patching> >(NULL, {"patches"});
  S.connect();

  cout <<S <<endl;

  engine().open(S);
  engine().waitForShutdownSignal();
  engine().close(S);

  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv) {
  mlr::initCmdLine(argc,argv);

  testModuleVision();

  return 0;
}
